#include "global_mapper.h"

#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <string>

#include <ceres/rotation.h>
#include <Eigen/Core>
#include <glog/logging.h>

#include <colmap/base/correspondence_graph.h>
#include <colmap/base/image.h>
#include <colmap/util/misc.h>
#include <colmap/util/threading.h>

#include <gopt/graph/edge.h>
#include <gopt/geometry/rotation.h>
#include <gopt/geometry/rotation_utils.h>
#include <gopt/utils/random.h>
#include <gopt/utils/types.h>

namespace gopt {
namespace {
// Helper struct to maintain the graph for the translation projection problem.
struct MFASNode {
  std::unordered_map<image_t, double> incoming_nodes;
  std::unordered_map<image_t, double> outgoing_nodes;
  double incoming_weight = 0;
  double outgoing_weight = 0;
};

typedef std::pair<Eigen::Vector2d, Eigen::Vector2d> PointsPair;

// Find the next view to add to the order. We attempt to choose a source (i.e.,
// a node with no incoming edges) or choose a node based on a heuristic such
// that it has the most source-like properties.
image_t FindNextViewInOrder(
    const std::unordered_map<image_t, MFASNode>& degrees_for_view) {
  image_t best_choice = kInvalidImageId;
  double best_score = 0;
  for (const auto& view : degrees_for_view) {
    // If the view is a source view, return it.
    if (view.second.incoming_nodes.size() == 0) {
      return view.first;
    }

    // Otherwise, keep track of the max score seen so far.
    const double score = (view.second.outgoing_weight + 1.0) /
                         (view.second.incoming_weight + 1.0);
    if (score > best_score) {
      best_choice = view.first;
      best_score = score;
    }
  }

  return best_choice;
}

bool AngularDifferenceIsAcceptable(
    const Eigen::Vector3d& orientation1,
    const Eigen::Vector3d& orientation2,
    const Eigen::Vector3d& relative_orientation,
    const double sq_max_relative_rotation_difference_radians) {
  const Eigen::Vector3d composed_relative_rotation =
      geometry::MultiplyRotations(orientation2, -orientation1);
  const Eigen::Vector3d loop_rotation =
      geometry::MultiplyRotations(-relative_orientation, composed_relative_rotation);
  const double sq_rotation_angular_difference_radians =
      loop_rotation.squaredNorm();
  return sq_rotation_angular_difference_radians <=
         sq_max_relative_rotation_difference_radians;
}

void FilterViewPairsFromOrientation(
    const std::unordered_map<image_t, Eigen::Vector3d>& orientations,
    const double max_relative_rotation_difference_degrees,
    graph::ViewGraph* view_graph) {
  CHECK_NOTNULL(view_graph);
  CHECK_GE(max_relative_rotation_difference_degrees, 0.0);

  // Precompute the squared threshold in radians.
  const double max_relative_rotation_difference_radians =
      geometry::DegToRad(max_relative_rotation_difference_degrees);
  const double sq_max_relative_rotation_difference_radians =
      max_relative_rotation_difference_radians *
      max_relative_rotation_difference_radians;

  std::unordered_set<ImagePair> image_pairs_to_remove;
  const auto& edges = view_graph->GetAllEdgePairs();

  for (const auto& edge_iter : edges) {
    const ImagePair image_pair = edge_iter.first;

    const Eigen::Vector3d* orientation1 =
        FindOrNull(orientations, image_pair.first);
    const Eigen::Vector3d* orientation2 =
        FindOrNull(orientations, image_pair.second);

    // If the view pair contains a view that does not have an orientation then
    // remove it.
    if (orientation1 == nullptr || orientation2 == nullptr) {
      LOG(WARNING)
          << "View pair (" << image_pair.first << ", " << image_pair.second
          << ") contains a view that does not exist! Removing the view pair.";
      image_pairs_to_remove.insert(image_pair);
      continue;
    }

    // Remove the view pair if the relative rotation estimate is not within the
    // tolerance.
    if (!AngularDifferenceIsAcceptable(
            *orientation1,
            *orientation2,
            edge_iter.second.rel_rotation,
            sq_max_relative_rotation_difference_radians)) {
      image_pairs_to_remove.insert(image_pair);
    }
  }

  // Remove all the "bad" relative poses.
  for (const ImagePair& image_pair : image_pairs_to_remove) {
    view_graph->DeleteEdge(image_pair.first, image_pair.second);
  }
  VLOG(1) << "Removed " << image_pairs_to_remove.size()
          << " view pairs by rotation filtering.";
}

// Creates the constraint matrix such that ||A * t|| is minimized, where A is
// R_i * f_i x R_j * f_j. Given known rotations, we can solve for the
// relative translation from this constraint matrix.
void CreateConstraintMatrix(
    const std::vector<PointsPair>& correspondences,
    const Eigen::Vector3d& rotation1,
    const Eigen::Vector3d& rotation2,
    Eigen::MatrixXd* constraint_matrix) {
  constraint_matrix->resize(3, correspondences.size());

  Eigen::Matrix3d rotation_matrix1;
  ceres::AngleAxisToRotationMatrix(
      rotation1.data(), ceres::ColumnMajorAdapter3x3(rotation_matrix1.data()));
  Eigen::Matrix3d rotation_matrix2;
  ceres::AngleAxisToRotationMatrix(
      rotation2.data(), ceres::ColumnMajorAdapter3x3(rotation_matrix2.data()));

  for (size_t i = 0; i < correspondences.size(); i++) {
    const Eigen::Vector3d rotated_feature1 =
        rotation_matrix1.transpose() *
        correspondences[i].first.homogeneous();
    const Eigen::Vector3d rotated_feature2 =
        rotation_matrix2.transpose() *
        correspondences[i].second.homogeneous();

    constraint_matrix->col(i) =
        rotated_feature2.cross(rotated_feature1).transpose() *
        rotation_matrix1.transpose();
  }
}

bool IsTriangulatedPointInFrontOfCameras(
    const PointsPair& correspondence,
    const Eigen::Matrix3d& rotation,
    const Eigen::Vector3d& position) {
  const Eigen::Vector3d dir1 = correspondence.first.homogeneous();
  const Eigen::Vector3d dir2 =
      rotation.transpose() * correspondence.second.homogeneous();

  const double dir1_sq = dir1.squaredNorm();
  const double dir2_sq = dir2.squaredNorm();
  const double dir1_dir2 = dir1.dot(dir2);
  const double dir1_pos = dir1.dot(position);
  const double dir2_pos = dir2.dot(position);

  return (dir2_sq * dir1_pos - dir1_dir2 * dir2_pos > 0 &&
          dir1_dir2 * dir1_pos - dir1_sq * dir2_pos > 0);
}

// Determines if the majority of the points are in front of the cameras. This is
// useful for determining the sign of the relative position. Returns true if
// more than 50% of correspondences are in front of both cameras and false
// otherwise.
bool MajorityOfPointsInFrontOfCameras(
    const std::vector<PointsPair>& correspondences,
    const Eigen::Vector3d& rotation1,
    const Eigen::Vector3d& rotation2,
    const Eigen::Vector3d& relative_position) {
  // Compose the relative rotation.
  Eigen::Matrix3d rotation_matrix1, rotation_matrix2;
  ceres::AngleAxisToRotationMatrix(
      rotation1.data(), ceres::ColumnMajorAdapter3x3(rotation_matrix1.data()));
  ceres::AngleAxisToRotationMatrix(
      rotation2.data(), ceres::ColumnMajorAdapter3x3(rotation_matrix2.data()));
  const Eigen::Matrix3d relative_rotation_matrix =
      rotation_matrix2 * rotation_matrix1.transpose();

  // Tests all points for cheirality.
  size_t num_points_in_front_of_cameras = 0;
  for (const PointsPair& match : correspondences) {
    if (IsTriangulatedPointInFrontOfCameras(match,
                                            relative_rotation_matrix,
                                            relative_position)) {
      ++num_points_in_front_of_cameras;
    }
  }

  return num_points_in_front_of_cameras > (correspondences.size() / 2);
}

// Given known camera rotations and feature correspondences, this method solves
// for the relative translation that optimizes the epipolar error
// f_i * E * f_j^t = 0.
bool OptimizeRelativePositionWithKnownRotation(
    const std::vector<PointsPair>& correspondences,
    const Eigen::Vector3d& rotation1,
    const Eigen::Vector3d& rotation2,
    Eigen::Vector3d* relative_position) {
  CHECK_NOTNULL(relative_position);

  // Set the initial relative position to random. This helps avoid a bad local
  // minima that is achieved from poor initialization.
  relative_position->setRandom();

  // Constants used for the IRLS solving.
  const double eps = 1e-5;
  const int kMaxIterations = 100;
  const int kMaxInnerIterations = 10;
  const double kMinWeight = 1e-7;

  // Create the constraint matrix from the known correspondences and rotations.
  Eigen::MatrixXd constraint_matrix;
  CreateConstraintMatrix(correspondences,
                         rotation1,
                         rotation2,
                         &constraint_matrix);

  // Initialize the weighting terms for each correspondence.
  Eigen::VectorXd weights(correspondences.size());
  weights.setConstant(1.0);

  // Solve for the relative positions using a robust IRLS.
  double cost = 0;
  int num_inner_iterations = 0;
  for (int i = 0;
       i < kMaxIterations && num_inner_iterations < kMaxInnerIterations;
       i++) {
    // Limit the minimum weight at kMinWeight.
    weights = (weights.array() < kMinWeight).select(kMinWeight, weights);

    // Apply the weights to the constraint matrix.
    const Eigen::Matrix3d lhs = constraint_matrix *
                                weights.asDiagonal().inverse() *
                                constraint_matrix.transpose();

    // Solve for the relative position which is the null vector of the weighted
    // constraints.
    const Eigen::Vector3d new_relative_position =
        lhs.jacobiSvd(Eigen::ComputeFullU).matrixU().rightCols<1>();

    // Update the weights based on the current errors.
    weights =
        (new_relative_position.transpose() * constraint_matrix).array().abs();

    // Compute the new cost.
    const double new_cost = weights.sum();

    // Check for convergence.
    const double delta = std::max(std::abs(cost - new_cost),
                                  1 - new_relative_position.squaredNorm());

    // If we have good convergence, attempt an inner iteration.
    if (delta <= eps) {
      ++num_inner_iterations;
    } else {
      num_inner_iterations = 0;
    }

    cost = new_cost;
    *relative_position = new_relative_position;
  }

  // The position solver above does not consider the sign of the relative
  // position. We can determine the sign by choosing the sign that puts the most
  // points in front of the camera.
  if (!MajorityOfPointsInFrontOfCameras(correspondences,
                                        rotation1,
                                        rotation2,
                                        *relative_position)) {
    *relative_position *= -1.0;
  }

  return true;
}

void RefineRelativeTranslationsWithKnownRotations(
    const colmap::Reconstruction& reconstruction,
    const std::unordered_map<image_t, Eigen::Vector3d>& orientations,
    const int num_threads,
    const colmap::DatabaseCache& database_cache,
    graph::ViewGraph* view_graph) {
  const auto& image_pairs = view_graph->GetAllEdgePairs();

  // ThreadPool pool(num_threads);
  colmap::ThreadPool thread_pool(num_threads);
  // Refine the translation estimation for each view pair.
  for (const auto& image_pair : image_pairs) {
    // Get all feature correspondences common to both views.
    std::vector<PointsPair> normalized_matches;
    const image_t image_id1 = image_pair.first.first;
    const image_t image_id2 = image_pair.first.second;
    
    const colmap::Image& image1 = database_cache.Image(image_id1);
    const colmap::Image& image2 = database_cache.Image(image_id2);
    
    const colmap::Camera& camera1 = database_cache.Camera(image1.CameraId());
    const colmap::Camera& camera2 = database_cache.Camera(image2.CameraId());
    const colmap::CorrespondenceGraph& correspondence_graph =
        database_cache.CorrespondenceGraph();
    colmap::FeatureMatches matches =
        correspondence_graph.FindCorrespondencesBetweenImages(image_id1, image_id2);
    
    std::vector<Eigen::Vector2d> points1;
    points1.reserve(image1.NumPoints2D());
    for (const auto& point : image1.Points2D()) {
      points1.push_back(point.XY());
    }

    std::vector<Eigen::Vector2d> points2;
    points2.reserve(image2.NumPoints2D());
    for (const auto& point : image2.Points2D()) {
      points2.push_back(point.XY());
    }
    
    for (size_t i = 0; i < matches.size(); i++) {
      const colmap::point2D_t idx1 = matches[i].point2D_idx1;
      const colmap::point2D_t idx2 = matches[i].point2D_idx2;
      const Eigen::Vector2d point1 = points1[idx1];
      const Eigen::Vector2d point2 = points1[idx2];
      const Eigen::Vector2d normalized_point1 = camera1.ImageToWorld(point1);
      const Eigen::Vector2d normalized_point2 = camera2.ImageToWorld(point2);
      normalized_matches.emplace_back(normalized_point1, normalized_point2);
    }

    graph::ViewEdge& edge = view_graph->GetEdge(image_pair.first.first,
                                                image_pair.first.second);
    thread_pool.AddTask(OptimizeRelativePositionWithKnownRotation,
                        normalized_matches,
                        FindOrDie(orientations, image_pair.first.first),
                        FindOrDie(orientations, image_pair.first.second),
                        &edge.rel_translation);
  }
  thread_pool.Wait();
}

// Rotate the translation direction based on the known orientation such that the
// translation is in the global reference frame.
std::unordered_map<ImagePair, Eigen::Vector3d>
RotateRelativeTranslationsToGlobalFrame(
    const std::unordered_map<image_t, Eigen::Vector3d>& orientations,
    const std::unordered_map<ImagePair, graph::ViewEdge>& image_pairs) {
  std::unordered_map<ImagePair, Eigen::Vector3d> rotated_translations;
  rotated_translations.reserve(orientations.size());

  for (const auto& image_pair : image_pairs) {
    const Eigen::Vector3d view_to_world_rotation =
        -1.0 * FindOrDie(orientations, image_pair.first.first);
    Eigen::Vector3d rotated_translation;
    ceres::AngleAxisRotatePoint(view_to_world_rotation.data(),
                                image_pair.second.rel_translation.data(),
                                rotated_translation.data());
    rotated_translations.emplace(image_pair.first, rotated_translation);
  }
  return rotated_translations;
}


// This chooses a random axis based on the given relative translations.
void ComputeMeanVariance(
    const std::unordered_map<ImagePair, Eigen::Vector3d>& relative_translations,
    Eigen::Vector3d* mean,
    Eigen::Vector3d* variance) {
  mean->setZero();
  variance->setZero();
  for (const auto& translation : relative_translations) {
    *mean += translation.second;
  }
  *mean /= static_cast<double>(relative_translations.size());

  for (const auto& translation : relative_translations) {
    *variance += (translation.second - *mean).cwiseAbs2();
  }
  *variance /= static_cast<double>(relative_translations.size() - 1);
}

// Based on the 1D translation projections, compute an ordering of the
// translations.
std::unordered_map<image_t, int> OrderTranslationsFromProjections(
    const std::unordered_map<ImagePair, double>&
        translation_direction_projections) {
  // Compute the degrees of all vertices as the sum of weights coming in or out.
  std::unordered_map<image_t, MFASNode> degrees_for_view;
  for (const auto& translation_projection : translation_direction_projections) {
    const ImagePair image_pair =
        (translation_projection.second > 0)
            ? translation_projection.first
            : ImagePair(translation_projection.first.second,
                         translation_projection.first.first);

    // Update the MFAS entry.
    const double weight = std::abs(translation_projection.second);
    degrees_for_view[image_pair.second].incoming_weight += weight;
    degrees_for_view[image_pair.first].outgoing_weight += weight;
    degrees_for_view[image_pair.second].incoming_nodes.emplace(
        image_pair.first, weight);
    degrees_for_view[image_pair.first].outgoing_nodes.emplace(
        image_pair.second, weight);
  }

  // Compute the ordering.
  const int num_views = degrees_for_view.size();
  std::unordered_map<image_t, int> translation_ordering;
  for (int i = 0; i < num_views; i++) {
    // Find the next view to add.
    const image_t next_view_in_order = FindNextViewInOrder(degrees_for_view);
    translation_ordering[next_view_in_order] = i;

    // Update the MFAS graph and remove the next view from the degrees_for_view.
    const auto& next_view_info =
        FindOrDie(degrees_for_view, next_view_in_order);
    for (auto& neighbor_info : next_view_info.incoming_nodes) {
      degrees_for_view[neighbor_info.first].outgoing_weight -=
          neighbor_info.second;
      degrees_for_view[neighbor_info.first].outgoing_nodes.erase(
          next_view_in_order);
    }
    for (auto& neighbor_info : next_view_info.outgoing_nodes) {
      degrees_for_view[neighbor_info.first].incoming_weight -=
          neighbor_info.second;
      degrees_for_view[neighbor_info.first].incoming_nodes.erase(
          next_view_in_order);
    }
    degrees_for_view.erase(next_view_in_order);
  }

  return translation_ordering;
}

// Projects all the of the translation onto the given axis.
std::unordered_map<ImagePair, double> ProjectTranslationsOntoAxis(
    const Eigen::Vector3d& axis,
    const std::unordered_map<ImagePair, Eigen::Vector3d>& relative_translations) {
  std::unordered_map<ImagePair, double> projection_weights;
  projection_weights.reserve(relative_translations.size());

  for (const auto& relative_translation : relative_translations) {
    const double projection_weight = relative_translation.second.dot(axis);
    projection_weights.emplace(relative_translation.first, projection_weight);
  }
  return projection_weights;
}

// Performs a single iterations of the translation filtering. This method is
// thread-safe.
void TranslationFilteringIteration(
    const std::unordered_map<ImagePair, Eigen::Vector3d>& relative_translations,
    const Eigen::Vector3d& direction_mean,
    const Eigen::Vector3d& direction_variance,
    const std::shared_ptr<RandomNumberGenerator>& rng,
    std::mutex* mutex,
    std::unordered_map<ImagePair, double>* bad_edge_weight) {
  // Create the random number generator within each thread. If the random number
  // generator is not supplied then create a new one within each thread.
  std::shared_ptr<RandomNumberGenerator> local_rng;
  if (rng.get() == nullptr) {
    local_rng = std::make_shared<RandomNumberGenerator>();
  } else {
    local_rng = rng;
  }

  // Get a random vector to project all relative translations on to.
  const Eigen::Vector3d random_axis =
      Eigen::Vector3d(
          local_rng->RandGaussian(direction_mean[0], direction_variance[0]),
          local_rng->RandGaussian(direction_mean[1], direction_variance[1]),
          local_rng->RandGaussian(direction_mean[2], direction_variance[2]))
          .normalized();

  // Project all vectors.
  const std::unordered_map<ImagePair, double>&
      translation_direction_projections =
          ProjectTranslationsOntoAxis(random_axis, relative_translations);

  // Compute ordering.
  const std::unordered_map<image_t, int>& translation_ordering =
      OrderTranslationsFromProjections(translation_direction_projections);

  // Compute bad edge weights.
  for (auto& edge : *bad_edge_weight) {
    const int ordering_diff =
        FindOrDie(translation_ordering, edge.first.second) -
        FindOrDie(translation_ordering, edge.first.first);
    const double& projection_weight_of_edge =
        FindOrDieNoPrint(translation_direction_projections, edge.first);

    VLOG(3) << "Edge (" << edge.first.first << ", " << edge.first.second
            << ") has ordering diff of " << ordering_diff
            << " and a projection of " << projection_weight_of_edge << " from "
            << FindOrDieNoPrint(relative_translations, edge.first).transpose();
    // If the ordering is inconsistent, add the absolute value of the bad weight
    // to the aggregate bad weight.
    if ((ordering_diff < 0 && projection_weight_of_edge > 0) ||
        (ordering_diff > 0 && projection_weight_of_edge < 0)) {
      std::lock_guard<std::mutex> lock(*mutex);
      edge.second += std::abs(projection_weight_of_edge);
    }
  }
}

void FilterViewPairsFromRelativeTranslation(
    const std::unordered_map<image_t, Eigen::Vector3d>& orientations,
    const int num_threads,
    const int num_iterations,
    const double translation_projection_tolerance,
    graph::ViewGraph* view_graph) {
  const auto& image_pairs = view_graph->GetAllEdgePairs();
  std::shared_ptr<RandomNumberGenerator> rng;

  // Weights of edges that have been accumulated throughout the iterations. A
  // higher weight means the edge is more likely to be bad.
  std::unordered_map<ImagePair, double> bad_edge_weight;
  for (const auto& image_pair : image_pairs) {
    bad_edge_weight[image_pair.first] = 0.0;
  }

  // Compute the adjusted translations so that they are oriented in the global
  // frame.
  const std::unordered_map<ImagePair, Eigen::Vector3d>& rotated_translations =
      RotateRelativeTranslationsToGlobalFrame(orientations, image_pairs);

  Eigen::Vector3d translation_mean, translation_variance;
  ComputeMeanVariance(rotated_translations,
                      &translation_mean,
                      &translation_variance);

  colmap::ThreadPool thread_pool(num_threads);
  std::mutex mutex;
  for (int i = 0; i < num_iterations; i++) {
    thread_pool.AddTask(
      TranslationFilteringIteration,
      rotated_translations,
      translation_mean,
      translation_variance,
      rng,
      &mutex,
      &bad_edge_weight);
  }
  // Wait for tasks to finish.
  thread_pool.Wait();

  // Remove all the bad edges.
  const double max_aggregated_projection_tolerance =
      translation_projection_tolerance * num_iterations;
  int num_image_pairs_removed = 0;
  for (const auto& image_pair : bad_edge_weight) {
    VLOG(3) << "View pair (" << image_pair.first.first << ", "
            << image_pair.first.second << ") projection = " << image_pair.second;
    if (image_pair.second > max_aggregated_projection_tolerance) {
      view_graph->DeleteEdge(image_pair.first.first, image_pair.first.second);
      ++num_image_pairs_removed;
    }
  }

  VLOG(1) << "Removed " << num_image_pairs_removed
          << " view pairs by relative translation filtering.";
}

size_t CompleteAndMergeTracks(
    const colmap::IncrementalTriangulator::Options& tri_options,
    GlobalMapper* mapper) {
  const size_t num_completed_observations = mapper->CompleteTracks(tri_options);
  std::cout << "  => Completed observations: " << num_completed_observations
            << std::endl;
  const size_t num_merged_observations = mapper->MergeTracks(tri_options);
  std::cout << "  => Merged observations: " << num_merged_observations
            << std::endl;
  return num_completed_observations + num_merged_observations;
}

}  // namespace

colmap::IncrementalTriangulator::Options GlobalMapper::Options::Triangulation() const {
  colmap::IncrementalTriangulator::Options options = triangulation;
  options.min_focal_length_ratio = min_focal_length_ratio;
  options.max_focal_length_ratio = max_focal_length_ratio;
  options.max_extra_param = max_extra_param;
  return options;
}

colmap::BundleAdjustmentOptions GlobalMapper::Options::GlobalBundleAdjustment()
    const {
  colmap::BundleAdjustmentOptions options;
  options.solver_options.function_tolerance = ba_global_function_tolerance;
  options.solver_options.gradient_tolerance = 1.0;
  options.solver_options.parameter_tolerance = 0.0;
  options.solver_options.max_num_iterations = ba_global_max_num_iterations;
  options.solver_options.max_linear_solver_iterations = 100;
  options.solver_options.minimizer_progress_to_stdout = false;
  options.solver_options.num_threads = num_threads;
#if CERES_VERSION_MAJOR < 2
  options.solver_options.num_linear_solver_threads = num_threads;
#endif  // CERES_VERSION_MAJOR
  options.print_summary = true;
  options.refine_focal_length = ba_refine_focal_length;
  options.refine_principal_point = ba_refine_principal_point;
  options.refine_extra_params = ba_refine_extra_params;
  options.min_num_residuals_for_multi_threading =
      ba_min_num_residuals_for_multi_threading;
  options.loss_function_type =
      colmap::BundleAdjustmentOptions::LossFunctionType::SOFT_L1;
  return options;
}

GlobalMapper::GlobalMapper(const GlobalMapper::Options& options,
                           graph::ViewGraph* view_graph,
                           const colmap::DatabaseCache* database_cache)
    : options_(options),
      database_cache_(database_cache),
      reconstruction_(nullptr),
      view_graph_(view_graph),
      triangulator_(nullptr) {}

void GlobalMapper::BeginReconstruction(colmap::Reconstruction* reconstruction) {
  CHECK(reconstruction_ == nullptr);
  reconstruction_ = reconstruction;
  reconstruction_->Load(*database_cache_);
  reconstruction_->SetUp(&database_cache_->CorrespondenceGraph());
  triangulator_.reset(new colmap::IncrementalTriangulator(
      &database_cache_->CorrespondenceGraph(), reconstruction));

  num_reg_images_per_camera_.clear();
  for (const image_t image_id : reconstruction_->RegImageIds()) {
    RegisterImageEvent(image_id);
  }
}

void GlobalMapper::EndReconstruction(const bool discard) {
  CHECK_NOTNULL(reconstruction_);

  if (discard) {
    for (const image_t image_id : reconstruction_->RegImageIds()) {
      DeRegisterImageEvent(image_id);
    }
  }

  reconstruction_->TearDown();
  reconstruction_ = nullptr;
  triangulator_.reset();
}

void GlobalMapper::Run(
    const RotationEstimatorOptions& rotation_estimator_options,
    const PositionEstimatorOptions& position_estimator_options) {
  // 1. Filter the initial view graph and remove any bad two view geometries.
  if (!FilterInitialViewGraph()) {
    LOG(INFO) << "Insufficient view pairs to perform estimation.";
  }

  // 2. Estimate global rotations and positions.
  view_graph_->RotationAveraging(
      rotation_estimator_options, &global_rotations_);

  // 3. Filter bad rotations.
  FilterRotations();

  // 4. Optimize relative translation.
  OptimizePairwiseTranslations();

  // 5. Filter bad relative translations.
  FilterRelativeTranslation();

  // 6. Estimate global positions.
  view_graph_->TranslationAveraging(
      position_estimator_options, &global_positions_);

  // 7. Register images and camera poses.
  for (const auto& iter : global_rotations_) {
    const image_t image_id = iter.first;
    const Eigen::Vector3d& angle_axis = global_rotations_.at(image_id);
    const Eigen::Vector3d& center = global_positions_.at(image_id);
    const Eigen::Vector4d qvec =
        NormalizeQuaternion(AngleAxisToQuaternion(angle_axis));
    const Eigen::Matrix3d R = AngleAxisToRotationMatrix(angle_axis);
    const Eigen::Vector3d tvec = -R * center;
    
    colmap::Image& image = reconstruction_->Image(image_id);
    image.SetQvec(qvec);
    image.SetTvec(tvec);
    reconstruction_->RegisterImage(image_id);
  }

  // 8. Triangulation.
  Triangulation();
}

void GlobalMapper::Triangulation() {
  //////////////////////////////////////////////////////////////////////////////
  // Triangulation
  //////////////////////////////////////////////////////////////////////////////

  const auto tri_options = options_.Triangulation();

  const auto& reg_image_ids = reconstruction_->RegImageIds();

  for (size_t i = 0; i < reg_image_ids.size(); ++i) {
    const image_t image_id = reg_image_ids[i];

    const auto& image = reconstruction_->Image(image_id);

    colmap::PrintHeading1(colmap::StringPrintf(
        "Triangulating image #%d (%d)", image_id, i));

    const size_t num_existing_points3D = image.NumPoints3D();

    std::cout << "  => Image sees " << num_existing_points3D << " / "
              << image.NumObservations() << " points" << std::endl;

    TriangulateImage(tri_options, image_id);

    std::cout << "  => Triangulated "
              << (image.NumPoints3D() - num_existing_points3D) << " points"
              << std::endl;
  }

  //////////////////////////////////////////////////////////////////////////////
  // Retriangulation
  //////////////////////////////////////////////////////////////////////////////

  colmap::PrintHeading1("Retriangulation");

  CompleteAndMergeTracks(tri_options, this);

  //////////////////////////////////////////////////////////////////////////////
  // Bundle adjustment
  //////////////////////////////////////////////////////////////////////////////

  auto ba_options = options_.GlobalBundleAdjustment();
  // ba_options.refine_focal_length = false;
  // ba_options.refine_principal_point = false;
  // ba_options.refine_extra_params = false;

  // Configure bundle adjustment.
  colmap::BundleAdjustmentConfig ba_config;
  for (const image_t image_id : reconstruction_->RegImageIds()) {
    ba_config.AddImage(image_id);
  }

  for (int i = 0; i < options_.ba_global_max_refinements; ++i) {
    // Avoid degeneracies in bundle adjustment.
    reconstruction_->FilterObservationsWithNegativeDepth();

    const size_t num_observations = reconstruction_->ComputeNumObservations();

    colmap::PrintHeading1("Bundle adjustment");
    colmap::BundleAdjuster bundle_adjuster(ba_options, ba_config);
    CHECK(bundle_adjuster.Solve(reconstruction_));

    size_t num_changed_observations = 0;
    num_changed_observations += CompleteAndMergeTracks(tri_options, this);
    num_changed_observations += FilterPoints(options_);
    const double changed =
        static_cast<double>(num_changed_observations) / num_observations;
    std::cout << colmap::StringPrintf("  => Changed observations: %.6f", changed)
              << std::endl;
  }

  // Normalize scene for numerical stability and
  // to avoid large scale changes in viewer.
  reconstruction_->Normalize();

  colmap::PrintHeading1("Extracting colors");
  reconstruction_->ExtractColorsForAllImages(options_.image_path);
}

bool GlobalMapper::FilterInitialViewGraph() {
  // Remove any view pairs that do not have a sufficient number of inliers.
  std::unordered_set<ImagePair> image_pairs_to_remove;
  const auto& image_pairs = view_graph_->GetAllEdgePairs();
  for (const auto& image_pair : image_pairs) {
    if (image_pair.second.weight < options_.min_num_two_view_inliers) {
      image_pairs_to_remove.insert(image_pair.first);
    }
  }
  for (const ImagePair& image_pair : image_pairs_to_remove) {
    view_graph_->DeleteEdge(image_pair.first, image_pair.second);
  }

  // // Only reconstruct the largest connected component.
  // RemoveDisconnectedViewPairs(view_graph_);
  return view_graph_->GetEdgesNum() >= 1;
}

void GlobalMapper::FilterRotations() {
  // Filter view pairs based on the relative rotation and the estimated global
  // orientations.
  FilterViewPairsFromOrientation(
      global_rotations_,
      options_.rotation_filtering_max_difference_degrees,
      view_graph_);
  // // Remove any disconnected views from the estimation.
  // const std::unordered_set<image_t> removed_views =
  //     RemoveDisconnectedViewPairs(view_graph_);
  // for (const image_t removed_view : removed_views) {
  //   global_rotations_.erase(removed_view);
  // }
}

void GlobalMapper::OptimizePairwiseTranslations() {
  RefineRelativeTranslationsWithKnownRotations(*reconstruction_,
                                               global_rotations_,
                                               options_.num_threads,
                                               *database_cache_,
                                               view_graph_);
}

void GlobalMapper::FilterRelativeTranslation() {
  // Filter potentially bad relative translations.
  LOG(INFO) << "Filtering relative translations with 1DSfM filter.";
  FilterViewPairsFromRelativeTranslation(global_rotations_,
                                         options_.num_threads,
                                         options_.num_iterations,
                                         options_.translation_projection_tolerance,
                                         view_graph_);

  // // Remove any disconnected views from the estimation.
  // const std::unordered_set<image_t> removed_views =
  //     RemoveDisconnectedViewPairs(view_graph_);
  // for (const image_t removed_view : removed_views) {
  //   global_rotations_.erase(removed_view);
  // }
}

size_t GlobalMapper::FilterPoints(const Options& options) {
  CHECK_NOTNULL(reconstruction_);
  // CHECK(options.Check());
  return reconstruction_->FilterAllPoints3D(options.filter_max_reproj_error,
                                            options.filter_min_tri_angle);
}

size_t GlobalMapper::CompleteTracks(
    const colmap::IncrementalTriangulator::Options& tri_options) {
  CHECK_NOTNULL(reconstruction_);
  return triangulator_->CompleteAllTracks(tri_options);
}

size_t GlobalMapper::MergeTracks(
    const colmap::IncrementalTriangulator::Options& tri_options) {
  CHECK_NOTNULL(reconstruction_);
  return triangulator_->MergeAllTracks(tri_options);
}

size_t GlobalMapper::TriangulateImage(
    const colmap::IncrementalTriangulator::Options& tri_options,
    const image_t image_id) {
  CHECK_NOTNULL(reconstruction_);
  return triangulator_->TriangulateImage(tri_options, image_id);
}

void GlobalMapper::RegisterImageEvent(const image_t image_id) {
  const colmap::Image& image = reconstruction_->Image(image_id);
  size_t& num_reg_images_for_camera =
      num_reg_images_per_camera_[image.CameraId()];
  num_reg_images_for_camera += 1;

  size_t& num_regs_for_image = num_registrations_[image_id];
  num_regs_for_image += 1;
  if (num_regs_for_image == 1) {
    num_total_reg_images_ += 1;
  } else if (num_regs_for_image > 1) {
    num_shared_reg_images_ += 1;
  }
}

void GlobalMapper::DeRegisterImageEvent(const image_t image_id) {
  const colmap::Image& image = reconstruction_->Image(image_id);
  size_t& num_reg_images_for_camera =
      num_reg_images_per_camera_.at(image.CameraId());
  CHECK_GT(num_reg_images_for_camera, 0);
  num_reg_images_for_camera -= 1;

  size_t& num_regs_for_image = num_registrations_[image_id];
  num_regs_for_image -= 1;
  if (num_regs_for_image == 0) {
    num_total_reg_images_ -= 1;
  } else if (num_regs_for_image > 0) {
    num_shared_reg_images_ -= 1;
  }
}

} // namespace gopt
