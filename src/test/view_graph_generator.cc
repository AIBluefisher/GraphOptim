#include "test/view_graph_generator.h"

#include <cmath>
#include <vector>
#include <algorithm>

#include <glog/logging.h>

#include "geometry/align_point_clouds.h"
#include "geometry/rotation_utils.h"
#include "geometry/rotation.h"
#include "util/threading.h"

namespace gopt {

namespace {

template<typename T>
std::vector<size_t> ArgSort(const std::vector<T>& array) {
  std::vector<size_t> indices(array.size());
  std::iota(indices.begin(), indices.end(), 0);
  std::sort(indices.begin(), indices.end(),
            [&array](int left, int right) -> bool {
                // sort indices according to corresponding array element
                return array[left] < array[right];
            });

  return indices;
}

std::vector<size_t> ArgSort(const std::vector<graph::ViewEdge>& array) {
  std::vector<size_t> indices(array.size());
  std::iota(indices.begin(), indices.end(), 0);
  std::sort(indices.begin(), indices.end(),
            [&array](int left, int right) -> bool {
                // sort indices according to corresponding array element
                return array[left].weight < array[right].weight;
            });

  return indices;
}

// Aligns positions to the ground truth positions via a similarity
// transformation.
void AlignPositions(
    const std::unordered_map<image_t, Eigen::Vector3d>& gt_positions,
    std::unordered_map<image_t, Eigen::Vector3d>* positions) {
  // Collect all positions into a vector.
  std::vector<Eigen::Vector3d> gt_pos, pos;
  for (const auto& gt_position : gt_positions) {
    gt_pos.push_back(gt_position.second);
    const Eigen::Vector3d& position = FindOrDie(*positions, gt_position.first);
    pos.push_back(position);
  }

  Eigen::Matrix3d rotation;
  Eigen::Vector3d translation;
  double scale;
  AlignPointCloudsUmeyama(pos, gt_pos, &rotation, &translation, &scale);

  // Apply the similarity transformation.
  for (auto& position : *positions) {
    position.second = scale * (rotation * position.second) + translation;
  }
}

// Append the three terms to the generated g2o files:
//  (1) initial poses;
//  (2) estimated poses come from the traditional motion averaging;
//  (3) outlier edges
void ExtendG2oFile(const std::string& filename,
    const std::unordered_map<gopt::image_t, Eigen::Vector3d>& init_rotations,
    const std::unordered_map<gopt::image_t, Eigen::Vector3d>& init_positions,
    const std::unordered_map<gopt::image_t, Eigen::Vector3d>& estimated_rotations,
    const std::unordered_map<gopt::image_t, Eigen::Vector3d>& estimated_positions,
    const std::vector<ImagePair>& rotation_outlier_indices,
    const std::vector<ImagePair>& translation_outlier_indices) {
  std::ofstream ofs(filename, std::ios::app);
  if (!ofs.is_open()) {
    LOG(ERROR) << "Cannot open file " << filename;
  }

  const size_t num_nodes = init_rotations.size();
  
  // Write init camera poses.
  for (size_t i = 0; i < num_nodes; i++) {
    const Eigen::Vector3d& rotation = init_rotations.at(i);
    const Eigen::Vector3d& position = init_positions.at(i);
    const Eigen::Vector4d qvec = AngleAxisToQuaternion(rotation);
    ofs << "INIT_SE3:QUAT " << i << " " << position[0] << " "
        << position[1] << " " << position[2] << " " << qvec[1] << " "
        << qvec[2] << " " << qvec[3] << " " << qvec[0] << std::endl;
  }

  // Write estimated camera poses.
  for (size_t i = 0; i < num_nodes; i++) {
    const Eigen::Vector3d& rotation = estimated_rotations.at(i);
    const Eigen::Vector3d& position = estimated_positions.at(i);
    const Eigen::Vector4d qvec = AngleAxisToQuaternion(rotation);
    ofs << "EST_SE3:QUAT " << i << " " << position[0] << " "
        << position[1] << " " << position[2] << " " << qvec[1] << " "
        << qvec[2] << " " << qvec[3] << " " << qvec[0] << std::endl;
  }

  // Write outlier indices.
  for (const auto& image_pair : rotation_outlier_indices) {
    ofs << "OUTLIER:ROT " << image_pair.first << " "
        << image_pair.second << std::endl;
  }
  for (const auto& image_pair : translation_outlier_indices) {
    ofs << "OUTLIER:TRA " << image_pair.first << " "
        << image_pair.second << std::endl;
  }
  ofs.close();
}

} // namespace

ViewGraphGenerator::ViewGraphGenerator(const ViewGraphGeneratorOptions& options)
    : options_(options) {

}

void ViewGraphGenerator::Run() {
  ThreadPool thread_pool(options_.num_threads);
  for (size_t i = 0; i < options_.num_scenes; i++) {
    thread_pool.AddTask(&ViewGraphGenerator::Generate, this, i);
  }
  thread_pool.Wait();
}

void ViewGraphGenerator::Generate(const size_t scene_idx) {
  const size_t num_additional_nodes =
      std::round(rng_.RandFloat(0, 1) * options_.nodes_exp_ratio);
  const double completeness_ratio =
      (1.0 + rng_.RandDouble(0, 1)) * options_.completeness_factor;
  const double rotation_sigma =
      (1.0 + rng_.RandDouble(0, 1)) * options_.rotation_sigma_factor;
  const double rotation_outlier_ratio =
      (1.0 + rng_.RandDouble(0, 1)) * options_.rotation_outlier_factor;
  const double translation_sigma =
      (1.0 + rng_.RandDouble(0, 1)) * options_.translation_sigma_factor;
  const double translation_outlier_ratio =
      (1.0 + rng_.RandDouble(0, 1)) * options_.translation_outlier_factor;

  const size_t num_nodes = num_additional_nodes + options_.min_num_nodes;
  std::unordered_map<gopt::image_t, Eigen::Vector3d> gt_rotations;
  std::unordered_map<gopt::image_t, Eigen::Vector3d> gt_positions;
  graph::ViewGraph view_graph = GenerateRandomGraph(
      num_nodes, completeness_ratio, rotation_sigma, rotation_outlier_ratio,
      translation_sigma, translation_outlier_ratio,
      &gt_rotations, &gt_positions);
    
  // Change the gauge freedom of ground truth rotations.
  const Eigen::Vector3d angle_axis0 = gt_rotations.at(0);
  for (image_t i = 0; i < num_nodes; i++) {
    const Eigen::Vector3d angle_axis = gt_rotations[i];
    gt_rotations[i] =
      geometry::RelativeRotationFromTwoRotations(angle_axis0, angle_axis);
  }

  LOG(INFO) << "Validating Input Data w.r.t. Ground Truth";
  CHECK_EQ(view_graph.ExtractConnectedComponents().size(), 1);
  std::unordered_map<image_t, Eigen::Vector3d> estimated_rotations;
  std::unordered_map<image_t, Eigen::Vector3d> estimated_positions;
  ValidateViewGraph(view_graph, num_nodes, gt_rotations, gt_positions,
                    &estimated_rotations, &estimated_positions);

  // The g2o file contains the ground-truth absolute camera poses and
  // the relative motions with noises and outliers.
  const std::string g2o_filename = options_.output_path + "/"
      "VG" + std::to_string(scene_idx) + "_N" +
      std::to_string(view_graph.GetNodesNum()) +
      "_M" + std::to_string(view_graph.GetEdgesNum()) + ".g2o";
  view_graph.WriteG2OFile(g2o_filename);

  std::unordered_map<image_t, Eigen::Vector3d> init_global_rotations;
  std::unordered_map<image_t, Eigen::Vector3d> init_global_positions;
  view_graph.InitializeGlobalRotationsFromMST(&init_global_rotations);
  view_graph.InitializeGlobalPositions(&init_global_positions);
    
  std::vector<ImagePair> rotation_outlier_indices;
  std::vector<ImagePair> translation_outlier_indices;
  ComputeOutliers(view_graph,
      options_.rotation_outlier_threshold,
      options_.translation_outlier_threshold,
      &rotation_outlier_indices, &translation_outlier_indices);

  // We also extend the g2o file to store more informations.
  ExtendG2oFile(g2o_filename,
                init_global_rotations, init_global_positions,
                estimated_rotations, estimated_positions,
                rotation_outlier_indices, translation_outlier_indices);
}

graph::ViewGraph ViewGraphGenerator::GenerateRandomGraph(
    const size_t num_nodes, const double completeness_ratio,
    const double rotation_sigma, const double rotation_outlier_ratio,
    const double translation_sigma, const double translation_outlier_ratio,
    std::unordered_map<gopt::image_t, Eigen::Vector3d>* gt_rotations,
    std::unordered_map<gopt::image_t, Eigen::Vector3d>* gt_positions) {
  graph::ViewGraph view_graph;
  graph::ViewNode first_node(0); // The rotation for the first node is identity.
  view_graph.AddNode(first_node);

  // 1. Generate absolute rotations.
  GenerateAbsolutePoses(num_nodes, gt_rotations, gt_positions);

  // 2. Generate relative rotations.
  std::vector<graph::ViewEdge> candidate_edges =
    GenerateRelativePoses(num_nodes, *gt_rotations, *gt_positions);

  // 3. Randomly selecting some edges for the view graph.
  AddEdgesToViewGraph(
    num_nodes, completeness_ratio, candidate_edges, &view_graph);

  // 4. Perturb ground truth relative rotations by random noise.
  PerturbViewGraphByNoise(rotation_sigma, translation_sigma, &view_graph);

  // 5. Randomly perturb relative rotations with outliers.
  AddOutliersToViewGraph(
    num_nodes, rotation_outlier_ratio, translation_outlier_ratio, &view_graph);

  return view_graph;
}

void ViewGraphGenerator::GenerateAbsolutePoses(
    const size_t num_nodes,
    std::unordered_map<gopt::image_t, Eigen::Vector3d>* gt_rotations,
    std::unordered_map<gopt::image_t, Eigen::Vector3d>* gt_positions) {
  std::vector<double> angles;
  std::unordered_map<size_t, Eigen::Vector3d> Rs; // Absolute rotations.
  (*gt_rotations)[0] = Eigen::Vector3d::Zero();
  (*gt_positions)[0] = Eigen::Vector3d::Zero();

  // 1. Generate absolute poses.
  for (image_t i = 1; i < num_nodes; i++) {
    Eigen::Vector3d angle_axis = rng_.RandnVector3d();
    angle_axis = angle_axis / angle_axis.norm() *
        M_PI * rng_.RandDouble(0, 1) / 18.0;
    
    double y = 2.0 * (rng_.RandDouble(0, 1) - 0.5);
    Eigen::Vector3d angle_axis2(y, std::sqrt(1.0 - y * y), 0.0);
    angle_axis2 = angle_axis2 / angle_axis2.norm() *
        M_PI * rng_.RandStdGaussian() / 4.0;

    angles.push_back(angle_axis2[1]);
    Rs[i] = geometry::RelativeRotationFromTwoRotations(angle_axis, angle_axis2);

    // TODO(chenyu): Revise the generation of absolute positions
    // (especially the axis).
    (*gt_positions)[i] = 10.0 * rng_.RandVector3d();
  }

  const std::vector<size_t> indices = ArgSort(angles);
  for (image_t i = 1; i < num_nodes; i++) {
    (*gt_rotations)[i] = Rs[indices[i]];
  }
}

std::vector<graph::ViewEdge> ViewGraphGenerator::GenerateRelativePoses(
    const size_t num_nodes,
    const std::unordered_map<gopt::image_t, Eigen::Vector3d>& gt_rotations,
    const std::unordered_map<gopt::image_t, Eigen::Vector3d>& gt_positions) {
  std::vector<graph::ViewEdge> candidate_edges;
  for (graph::node_t i = 0; i < num_nodes - 1; i++) {
    for (graph::node_t j = i + 1; j < std::min(
         static_cast<graph::node_t>(num_nodes), 2 * (i + 2)); j++) {
      graph::ViewEdge edge(i, j);
      const Eigen::Vector3d Rij = geometry::RelativeRotationFromTwoRotations(
          gt_rotations.at(i), gt_rotations.at(j));
      edge.weight = Rij.norm();
      edge.rel_rotation = Rij;
      edge.rel_translation = geometry::RelativeTranslationFromTwoPositions(
        gt_positions.at(i), gt_positions.at(j), gt_rotations.at(i));
      candidate_edges.emplace_back(edge);
    }
  }

  rng_.RandShuffle(&candidate_edges);

  return candidate_edges;
}

void ViewGraphGenerator::AddEdgesToViewGraph(
    const size_t num_nodes, const double completeness_ratio,
    std::vector<graph::ViewEdge> candidate_edges,
    graph::ViewGraph* view_graph) {
  graph::ViewGraph adj_matrix;
  const double threshold = 45.0 + rng_.RandDouble(0, 1) * 15.0;
  std::vector<graph::ViewEdge> edge_angles;
  size_t k = 0;
  size_t num_small_angles = 0;

  for (graph::node_t i = 0; i < num_nodes - 1; i++) {
    for (graph::node_t j = i + 1; j < std::min(
         static_cast<graph::node_t>(num_nodes), 2 * (i + 2)); j++) {
      const graph::ViewEdge& view_edge = candidate_edges[k];
      const double theta = view_edge.rel_rotation.norm();

      graph::ViewEdge edge = view_edge;
      edge.weight = geometry::RadToDeg(theta);

      adj_matrix.AddEdge(edge);

      if (edge.weight < threshold) {
        num_small_angles++;
      }
      k += 1;
    }
  }

  // (1) Firstly, we seek for a minimum spanning tree and add the edge
  // to view graph.
  const std::vector<graph::ViewEdge> mst_edges = adj_matrix.Kruskal();
  for (const auto& edge : mst_edges) {
    view_graph->AddEdge(edge);
    graph::ViewEdge& view_edge = adj_matrix.GetEdge(edge.src, edge.dst);
    // We masked out the edges in the adjacent matrix.
    view_edge.weight = std::numeric_limits<double>::max();
  }

  const auto& edges = adj_matrix.GetEdges();
  for (const auto& edge_iter : edges) {
    const auto& em = edge_iter.second;
    for (const auto& em_iter : em) {
      edge_angles.emplace_back(em_iter.second);
    }
  }

  // (2) Secondly, we add other edges to the view graph with a threshold.
  const double num_additional_edges = std::min(
      static_cast<size_t>(std::ceil(completeness_ratio * adj_matrix.GetEdgesNum())),
      num_small_angles - mst_edges.size());
  if (num_additional_edges > num_nodes - 1) {
    const std::vector<size_t> additional_edges_indices = ArgSort(edge_angles);
    for (size_t i = 0; i < num_additional_edges - num_nodes + 1; i++) {
      view_graph->AddEdge(edge_angles[additional_edges_indices[i]]);
    }
  }
}

void ViewGraphGenerator::PerturbViewGraphByNoise(
    const double rotation_sigma,
    const double translation_sigma,
    graph::ViewGraph* view_graph) {
  const double Sigma_rotation =
    geometry::DegToRad(rotation_sigma) / std::sqrt(3);
  const double Sigma_translation =
    geometry::DegToRad(translation_sigma) / std::sqrt(3);
  auto& view_edges = view_graph->GetEdges();
  
  for (auto& edge_iter : view_edges) {
    auto& em = edge_iter.second;
    for (auto& em_iter : em) {
      // (1) Perturb relative rotations.
      Eigen::Vector3d angle_axis_noise = rng_.RandnVector3d();
      angle_axis_noise = angle_axis_noise / angle_axis_noise.norm() *
                         Sigma_rotation * rng_.RandStdGaussian() / 10;
      const double y = 2.0 * (rng_.RandDouble(0, 1) - 0.5);
      Eigen::Vector3d angle_axis_noise2(y, std::sqrt(1 - y * y), 0);
      angle_axis_noise2 =
        angle_axis_noise2 * Sigma_rotation * rng_.RandStdGaussian();

      const Eigen::Vector3d& rel_angle_axis = em_iter.second.rel_rotation;
      const Eigen::Matrix3d rel_rotation =
          AngleAxisToRotationMatrix(rel_angle_axis);
      const Eigen::Matrix3d rotation_noise = AngleAxisToRotationMatrix(
          angle_axis_noise);
      const Eigen::Matrix3d rotation_noise2 = AngleAxisToRotationMatrix(
          angle_axis_noise2);

      const Eigen::Matrix3d composed_rotation =
          rel_rotation * rotation_noise2 * rotation_noise;
      em_iter.second.rel_rotation =
        gopt::RotationMatrixToAngleAxis(composed_rotation);

      // (2) Perturb relative translations.
      Eigen::Vector3d random_translation = rng_.RandnVector3d();
      const Eigen::Vector3d random_axis =
        random_translation / random_translation.norm();
      const double noise = rng_.RandStdGaussian() * Sigma_translation / 10;
      em_iter.second.rel_translation =
        geometry::RelativeTranslationFromTwoPositions(
          em_iter.second.rel_translation, random_axis, noise);
    }
  }
}

void ViewGraphGenerator::AddOutliersToViewGraph(
    const size_t num_nodes,
    const double rotation_outlier_ratio,
    const double translation_outlier_ratio,
    graph::ViewGraph* view_graph) {
  const size_t num_edges = view_graph->GetEdgesNum();
  // NOTE: In real world, the outlier ratio w.r.t. relative rotations and
  // relative translations are different while relative translations have
  // more outliers than relative rotations.
  const size_t num_rotation_outlier_edges =
    std::round(rotation_outlier_ratio * num_edges);
  const size_t num_translation_outlier_edges =
    std::round(translation_outlier_ratio * num_edges);
  const double kMaxRotAngle = 90;   // unit: deg.
  const double kMaxTransAngle = 120; // unit: deg.

  std::unordered_set<ImagePair> visited_view_pairs;
  while (visited_view_pairs.size() < num_rotation_outlier_edges) {
    ImagePair view_id_pair(rng_.RandInt(0, num_nodes - 1),
                           rng_.RandInt(0, num_nodes - 1));
    // Ensure the first id is smaller than second id &&
    // do not add the views that already exists.
    if (view_id_pair.first >= view_id_pair.second ||
        visited_view_pairs.count(view_id_pair) != 0 ||
        !view_graph->HasEdge(view_id_pair.first, view_id_pair.second)) {
      continue;
    }

    visited_view_pairs.insert(view_id_pair);

    graph::ViewEdge& view_edge = view_graph->GetEdge(
        view_id_pair.first, view_id_pair.second);
    const Eigen::Matrix3d rel_rotation =
          AngleAxisToRotationMatrix(view_edge.rel_rotation);
    
    const Eigen::Vector3d perturb_angle_axis =
        geometry::DegToRad(rng_.RandStdGaussian() * kMaxRotAngle) /
        std::sqrt(3.0) * rng_.RandnVector3d();
    const Eigen::Matrix3d perturb_rotation =
        AngleAxisToRotationMatrix(perturb_angle_axis);
    const Eigen::Matrix3d perturbed_rotation = rel_rotation * perturb_rotation;
    const Eigen::Vector3d perturbed_angle_axis =
        RotationMatrixToAngleAxis(perturbed_rotation);
    view_edge.rel_rotation = perturbed_angle_axis;

    const double noise =
      geometry::DegToRad(rng_.RandStdGaussian() * kMaxTransAngle) / std::sqrt(3);
    const Eigen::Vector3d random_axis = rng_.RandnVector3d();
    view_edge.rel_translation = geometry::RelativeTranslationFromTwoPositions(
      view_edge.rel_translation, random_axis, noise);
  }

  while (visited_view_pairs.size() < num_translation_outlier_edges) {
    ImagePair view_id_pair(rng_.RandInt(0, num_nodes - 1),
                           rng_.RandInt(0, num_nodes - 1));
    // Ensure the first id is smaller than second id &&
    // do not add the views that already exists.
    if (view_id_pair.first >= view_id_pair.second ||
        visited_view_pairs.count(view_id_pair) != 0 ||
        !view_graph->HasEdge(view_id_pair.first, view_id_pair.second)) {
      continue;
    }

    visited_view_pairs.insert(view_id_pair);

    graph::ViewEdge& view_edge = view_graph->GetEdge(
        view_id_pair.first, view_id_pair.second);

    const double noise = geometry::DegToRad(rng_.RandStdGaussian() * kMaxTransAngle) / std::sqrt(3);
    const Eigen::Vector3d random_axis = rng_.RandnVector3d();
    view_edge.rel_translation = geometry::RelativeTranslationFromTwoPositions(
      view_edge.rel_translation, random_axis, noise);
  }
}

void ViewGraphGenerator::ValidateViewGraph(
    graph::ViewGraph& view_graph,
    const size_t num_nodes,
    const std::unordered_map<image_t, Eigen::Vector3d>& gt_rotations,
    const std::unordered_map<image_t, Eigen::Vector3d>& gt_positions,
    std::unordered_map<image_t, Eigen::Vector3d>* estimated_rotations,
    std::unordered_map<image_t, Eigen::Vector3d>* estimated_positions) {
  // Validate positions.
  gopt::RotationEstimatorOptions options;
  options.estimator_type = gopt::GlobalRotationEstimatorType::ROBUST_L1L2;
  view_graph.RotationAveraging(options, estimated_rotations);

  LOG(INFO) << "Align the rotations and measure the error";
  // Align the rotations and measure the error.
  geometry::AlignOrientations(gt_rotations, estimated_rotations);

  double sum_angular_error = 0.0;
  std::vector<double> angular_errors;

  for (size_t i = 0; i < gt_rotations.size(); i++) {
    const auto& rotation = gt_rotations.at(i);
    const Eigen::Vector3d& estimated_rotation = FindOrDie(*estimated_rotations, i);
    const Eigen::Vector3d relative_rotation =
        geometry::RelativeRotationFromTwoRotations(estimated_rotation,
                                                   rotation, 0.0);
    const double angular_error = geometry::RadToDeg(relative_rotation.norm());

    sum_angular_error += angular_error;
    angular_errors.push_back(angular_error);
  }

  std::sort(angular_errors.begin(), angular_errors.end());

  std::cout << "\n";
  LOG(INFO) << "Mean Angular Residual (deg): "
            << sum_angular_error / angular_errors.size();
  LOG(INFO) << "Median Angular Residual (deg): " << angular_errors[num_nodes / 2];
  LOG(INFO) << "Max Angular Residual (deg): " << angular_errors[num_nodes - 1];
  LOG(INFO) << "Min Angular Residual (deg): " << angular_errors[0];

  // Validate positions.
  gopt::PositionEstimatorOptions position_options;
  view_graph.TranslationAveraging(position_options, estimated_positions);

  // Align the positions and measure the error.
  AlignPositions(gt_positions, estimated_positions);

  std::vector<double> position_errors;
  for (const auto& position : gt_positions) {
    const Eigen::Vector3d& estimated_position =
        FindOrDie(*estimated_positions, position.first);
    const double position_error =
        (position.second - estimated_position).norm();
    position_errors.push_back(position_error);
  }

  double sum_errors = 0;
  for (const double error : position_errors) {
    sum_errors += error;
  }
  std::sort(position_errors.begin(), position_errors.end());

  const double mean_error = sum_errors / position_errors.size();
  LOG(INFO) << "Mean Position error: " << mean_error;
  LOG(INFO) << "Median Position error: " << position_errors[position_errors.size() / 2];

  // Overwrite the nodes' global poses by ground truth.
  for (image_t i = 0; i < num_nodes; i++) {
    auto& node = view_graph.GetNode(i);
    node.rotation = gt_rotations.at(i);
    node.position = gt_positions.at(i);
  }
}

void ViewGraphGenerator::ComputeOutliers(
    const graph::ViewGraph& view_graph,
    const double rotation_threshold,
    const double translation_threshold,
    std::vector<ImagePair>* rotation_outlier_indices,
    std::vector<ImagePair>* translation_outlier_indices) const {
  const auto& edges = view_graph.GetEdges();
  
  for (const auto& edge_iter : edges) {
    const auto& em = edge_iter.second;
    for (const auto& em_iter : em) {
      // outliers w.r.t. relative rotations.
      const Eigen::Vector3d& rel_rotation = em_iter.second.rel_rotation;
      const image_t src = em_iter.second.src;
      const image_t dst = em_iter.second.dst;
      const Eigen::Vector3d& rotation1 = view_graph.GetNode(src).rotation;
      const Eigen::Vector3d& rotation2 = view_graph.GetNode(dst).rotation;
      const Eigen::Vector3d rel_rotation_hat =
        geometry::RelativeRotationFromTwoRotations(rotation1, rotation2);
      const double rotation_angular_error =
        geometry::RelativeRotationFromTwoRotations(
          rel_rotation, rel_rotation_hat).norm();
      if (rotation_angular_error > rotation_threshold) {
        (*rotation_outlier_indices).emplace_back(src, dst);
      }
      
      // outliers w.r.t. relative translations.
      const Eigen::Vector3d& rel_translation = em_iter.second.rel_translation;
      const Eigen::Vector3d& position1 = view_graph.GetNode(src).position;
      const Eigen::Vector3d& position2 = view_graph.GetNode(dst).position;
      const Eigen::Vector3d rel_translation_hat = 
        geometry::RelativeTranslationFromTwoPositions(
          position1, position2, rotation1).normalized();
      const double translation_angular_error =
        std::acos(DotProduct(rel_translation, rel_translation_hat));
      if (translation_angular_error > translation_threshold) {
        (*translation_outlier_indices).emplace_back(src, dst);
      }
    }
  }
}

}  // namespace gopt
