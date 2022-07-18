#include "global_mapper_controller.h"

#include <colmap/base/image.h>
#include <colmap/base/reconstruction_manager.h>
#include <colmap/util/logging.h>
#include <colmap/util/misc.h>
#include <colmap/util/timer.h>

#include <gopt/graph/edge.h>
#include <gopt/geometry/rotation.h>
#include <gopt/geometry/track_builder.h>
#include <glog/logging.h>

#include "global_mapper.h"
#include "utils.h"

namespace gopt {

GlobalMapper::Options GlobalMapperOptions::Mapper() const {
  GlobalMapper::Options options = mapper;
  options.ba_refine_focal_length = ba_refine_focal_length;
  options.ba_refine_extra_params = ba_refine_extra_params;
  options.min_focal_length_ratio = min_focal_length_ratio;
  options.max_focal_length_ratio = max_focal_length_ratio;
  options.max_extra_param = max_extra_param;
  options.num_threads = num_threads;
  return options;
}

GlobalMapper::Options& GlobalMapperOptions::Mapper() {
  return mapper;
}

const RotationEstimatorOptions& GlobalMapperOptions::RotationEstimator() const {
  return rotation_estimator_options;
}

RotationEstimatorOptions& GlobalMapperOptions::RotationEstimator() {
  return rotation_estimator_options;
}

const PositionEstimatorOptions& GlobalMapperOptions::PositionEstimator() const {
  return position_estimator_options;
}

PositionEstimatorOptions& GlobalMapperOptions::PositionEstimator() {
  return position_estimator_options;
}

bool GlobalMapperOptions::Check() const {
  CHECK_GT(min_num_matches, 0);
  CHECK_GT(min_focal_length_ratio, 0);
  CHECK_GT(max_focal_length_ratio, 0);
  CHECK_GE(max_extra_param, 0);
  // CHECK(Mapper().Check());
  return true;
}

GlobalMapperController::GlobalMapperController(
    const GlobalMapperOptions* options, const std::string& image_path,
    const std::string& database_path,
    colmap::ReconstructionManager* reconstruction_manager)
    : options_(options),
      image_path_(image_path),
      database_path_(database_path),
      reconstruction_manager_(reconstruction_manager) {
  CHECK(options_->Check());
}

void GlobalMapperController::Run() {
  if (!LoadDatabase()) {
    LOG(ERROR) << "Load Database Failed!";
    return;
  }

  if (!LoadTwoViewGeometries()) {
    LOG(ERROR) << "Load Two View Geometries Failed!";
    return;
  }

  GlobalMapper::Options init_mapper_options = options_->Mapper();
  RotationEstimatorOptions rotation_options = options_->RotationEstimator();
  PositionEstimatorOptions position_options = options_->PositionEstimator();
  if (position_options.estimator_type == PositionEstimatorType::LIGT) {
    LoadTracks(&position_options.tracks);
    LoadNormalizedKeypoints(&position_options.normalized_keypoints);
  }
  LOG(INFO) << "tracks size: " << position_options.tracks.size();
  LOG(INFO) << "keypoints images: " << position_options.normalized_keypoints.size();

  init_mapper_options.image_path = image_path_;
  Reconstruct(init_mapper_options, rotation_options, position_options);

  std::cout << std::endl;
  GetTimer().PrintMinutes();
}

bool GlobalMapperController::LoadDatabase() {
  colmap::PrintHeading1("Loading database");

  // Make sure images of the given reconstruction are also included when
  // manually specifying images for the reconstruction procedure.
  std::unordered_set<std::string> image_names = options_->image_names;
  if (reconstruction_manager_->Size() == 1 && !options_->image_names.empty()) {
    const colmap::Reconstruction& reconstruction = reconstruction_manager_->Get(0);
    for (const image_t image_id : reconstruction.RegImageIds()) {
      const auto& image = reconstruction.Image(image_id);
      image_names.insert(image.Name());
    }
  }

  database_.Open(database_path_);
  colmap::Timer timer;
  timer.Start();
  const size_t min_num_matches = static_cast<size_t>(options_->min_num_matches);
  database_cache_.Load(database_, min_num_matches, options_->ignore_watermarks,
                       image_names);
  std::cout << std::endl;
  timer.PrintMinutes();

  std::cout << std::endl;

  if (database_cache_.NumImages() == 0) {
    LOG(WARNING) << "WARNING: No images with matches found in the database.";
    return false;
  }

  return true;
}

bool GlobalMapperController::LoadTwoViewGeometries() {
  // Loading database.
  colmap::Timer timer;

  // Reading all images.
  LOG(INFO) << "Reading images...";
  timer.Start();
  const std::vector<colmap::Image> vec_images = database_.ReadAllImages();

  for (const auto& image : vec_images) {
    graph::ViewNode node(image.ImageId(), image.Name());
    view_graph_.AddNode(node);
  }

  // Reading scene graph.
  LOG(INFO) << "Reading two view geometries...";
  std::vector<colmap::TwoViewGeometry> two_view_geometries;
  std::vector<image_pair_t> image_pair_ids;
  std::vector<std::pair<image_t, image_t>> image_pairs;
  std::vector<int> num_inliers;
  database_.ReadTwoViewGeometryNumInliers(&image_pairs, &num_inliers);
  database_.ReadTwoViewGeometries(&image_pair_ids, &two_view_geometries);

  CHECK_EQ(image_pairs.size(), image_pair_ids.size());

  LOG(INFO) << "Estimating two view geometries";
  // #pragma omp parallel for
  for (size_t i = 0; i < image_pairs.size(); i++) {
    const image_t image_id1 = image_pairs[i].first,
                  image_id2 = image_pairs[i].second;
    colmap::TwoViewGeometry two_view_geometry = two_view_geometries[i];

    graph::ViewEdge edge(image_id1, image_id2, two_view_geometry.inlier_matches.size());
    edge.rel_rotation = QuaternionToAngleAxis(two_view_geometry.qvec);
    edge.rel_translation = two_view_geometry.tvec;
    view_graph_.AddEdge(edge);
  }

  LOG(INFO) << "Nodes size: " << view_graph_.GetNodes().size();

  timer.Pause();
  LOG(INFO) << "Elapsed Time[database reading]: " << timer.ElapsedSeconds()
            << " seconds";
  LOG(INFO) << "image pairs: " << view_graph_.GetEdgesNum();

  return view_graph_.GetEdgesNum() > 0;
}

bool GlobalMapperController::LoadTracks(std::vector<TrackElements>* tracks) {
  TrackElements track_elements;
  std::vector<std::pair<track_t, track_t>> track_element_pairs;
  LoadTracksFromDB(database_path_, &track_elements, &track_element_pairs);

  TrackBuilder track_builder(options_->min_track_length,
                             options_->max_track_length);
  track_builder.Build(track_elements, track_element_pairs);
  track_builder.Filter();

  const auto& consistent_tracks = track_builder.GetConsistentTracks();
  const size_t num_tracks = consistent_tracks.size();
  tracks->reserve(num_tracks);
  for (const auto& iter : consistent_tracks) {
    tracks->push_back(iter.second);
  }
  return tracks->size() > 0;
}

bool GlobalMapperController::LoadNormalizedKeypoints(
    std::unordered_map<image_t, KeypointsMat>* normalized_keypoints) {
  const auto& images = database_cache_.Images();

  // Refine the translation estimation for each view pair.
  for (const auto& image_iter : images) {
    const image_t image_id = image_iter.first;
    const colmap::Image& image = image_iter.second;
    const colmap::Camera& camera = database_cache_.Camera(image.CameraId());
    
    KeypointsMat keypoints_mat = Eigen::MatrixXd::Zero(image.NumPoints2D(), 3);
    std::vector<colmap::Point2D> points2D = image.Points2D();
    for (size_t i = 0; i < points2D.size(); i++) {
      const Eigen::Vector2d normalized_point = camera.ImageToWorld(points2D[i].XY());
      keypoints_mat.row(i) = normalized_point.homogeneous().transpose();
    }
    (*normalized_keypoints).emplace(image_id, keypoints_mat);
  }
  return normalized_keypoints->size() > 0;
}

void GlobalMapperController::Reconstruct(
    const GlobalMapper::Options& init_mapper_options,
    const RotationEstimatorOptions& rotation_estimator_options,
    const PositionEstimatorOptions& position_estimator_options) {
  const bool kDiscardReconstruction = false;
  //////////////////////////////////////////////////////////////////////////////
  // Main loop
  //////////////////////////////////////////////////////////////////////////////

  GlobalMapper mapper(init_mapper_options, &view_graph_, &database_cache_);
  
  reconstruction_manager_->Add();
  colmap::Reconstruction& reconstruction = reconstruction_manager_->Get(0);
  mapper.BeginReconstruction(&reconstruction);
  mapper.Run(rotation_estimator_options, position_estimator_options);
  mapper.EndReconstruction(kDiscardReconstruction);

  reconstruction.Write(options_->output_path);
}

} // namespace gopt
