// BSD 3-Clause License

// Copyright (c) 2022, Chenyu
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.

// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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
