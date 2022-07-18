#ifndef APPLICATIONS_GLOBAL_MAPPER_CONTROLLER_H_
#define APPLICATIONS_GLOBAL_MAPPER_CONTROLLER_H_

#include <string>
#include <unordered_set>

#include <colmap/base/database.h>
#include <colmap/base/database_cache.h>
#include <colmap/base/reconstruction_manager.h>
#include <colmap/optim/bundle_adjustment.h>
#include <colmap/sfm/incremental_triangulator.h>
#include <colmap/util/threading.h>

#include <gopt/graph/view_graph.h>
#include <gopt/rotation_averaging/rotation_estimator.h>
#include <gopt/translation_averaging/position_estimator.h>

#include "global_mapper.h"

namespace gopt{

struct GlobalMapperOptions {
 public:
  std::string output_path;

  // The minimum number of matches for inlier matches to be considered.
  int min_num_matches = 15;

  size_t min_track_length = 3;
  size_t max_track_length = 30;

  // Whether to ignore the inlier matches of watermark image pairs.
  bool ignore_watermarks = false;

  // Whether to extract colors for reconstructed points.
  bool extract_colors = true;

  // The number of threads to use during reconstruction.
  int num_threads = -1;

  // Thresholds for filtering images with degenerate intrinsics.
  double min_focal_length_ratio = 0.1;
  double max_focal_length_ratio = 10.0;
  double max_extra_param = 1.0;

  // // Whether to use PBA in global bundle adjustment.
  // bool ba_global_use_pba = false;

  // // The GPU index for PBA bundle adjustment.
  // int ba_global_pba_gpu_index = -1;

  // Which intrinsic parameters to optimize during the reconstruction.
  bool ba_refine_focal_length = true;
  bool ba_refine_principal_point = false;
  bool ba_refine_extra_params = true;

  // Which images to reconstruct. If no images are specified, all images will
  // be reconstructed by default.
  std::unordered_set<std::string> image_names;

  const RotationEstimatorOptions& RotationEstimator() const;
  RotationEstimatorOptions& RotationEstimator();

  const PositionEstimatorOptions& PositionEstimator() const;
  PositionEstimatorOptions& PositionEstimator();
  
  GlobalMapper::Options Mapper() const;
  GlobalMapper::Options& Mapper();

  bool Check() const;

 private:
  friend class OptionManager;
  GlobalMapper::Options mapper;
  colmap::IncrementalTriangulator::Options triangulation;
  
  RotationEstimatorOptions rotation_estimator_options;
  PositionEstimatorOptions position_estimator_options;
};

class GlobalMapperController : public colmap::Thread {
 public:
  GlobalMapperController(const GlobalMapperOptions* options,
                         const std::string& image_path,
                         const std::string& database_path,
                         colmap::ReconstructionManager* reconstruction_manager);

 private:
  void Run();

  bool LoadDatabase();
  bool LoadTwoViewGeometries();
  bool LoadTracks(std::vector<TrackElements>* tracks);
  bool LoadNormalizedKeypoints(
      std::unordered_map<image_t, KeypointsMat>* normalized_keypoints);

  void Reconstruct(const GlobalMapper::Options& init_mapper_options,
                   const RotationEstimatorOptions& rotation_estimator_options,
                   const PositionEstimatorOptions& position_estimator_options);

  const GlobalMapperOptions* options_;
  const std::string image_path_;
  const std::string database_path_;
  
  colmap::ReconstructionManager* reconstruction_manager_;
  colmap::Database database_;
  colmap::DatabaseCache database_cache_;

  graph::ViewGraph view_graph_;
};

} // namespace gopt

#endif // APPLICATIONS_GLOBAL_MAPPER_CONTROLLER_H_
