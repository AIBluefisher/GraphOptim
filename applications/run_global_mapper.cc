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

#include <cstdlib>
#include <iostream>

#include <colmap/util/misc.h>
#include <colmap/util/option_manager.h>
#include <colmap/util/string.h>
#include <colmap/base/database.h>
#include <colmap/base/image.h>
#include <colmap/base/reconstruction_manager.h>
#include <colmap/controllers/incremental_mapper.h>
#include <colmap/base/essential_matrix.h>
#include <colmap/estimators/two_view_geometry.h>

#include <gopt/geometry/rotation.h>
#include <gopt/graph/view_graph.h>

#include "global_mapper_controller.h"

namespace {

bool LoadTwoViewGeometries(gopt::graph::ViewGraph* view_graph,
                           const std::string& database_path,
                           const colmap::Reconstruction& reconstruction) {
  // Loading database.
  colmap::Database database(database_path);

  // Reading all images.
  LOG(INFO) << "Reading images...";
  const std::vector<colmap::Image> vec_images = database.ReadAllImages();
  const auto& images = reconstruction.Images();

  for (const auto& image : vec_images) {
    const auto image_id = image.ImageId();
    gopt::graph::ViewNode node(image_id - 1, image.Name());
    node.rotation = gopt::QuaternionToAngleAxis(images.at(image_id).Qvec());
    node.position = images.at(image_id).ProjectionCenter();
    view_graph->AddNode(node);
  }

  // Reading scene graph.
  LOG(INFO) << "Reading two view geometries...";
  std::vector<colmap::TwoViewGeometry> two_view_geometries;
  std::vector<gopt::image_pair_t> image_pair_ids;
  std::vector<std::pair<gopt::image_t, gopt::image_t>> image_pairs;
  std::vector<int> num_inliers;
  database.ReadTwoViewGeometryNumInliers(&image_pairs, &num_inliers);
  database.ReadTwoViewGeometries(&image_pair_ids, &two_view_geometries);

  CHECK_EQ(image_pairs.size(), image_pair_ids.size());
  double eps = 1e-5;

  LOG(INFO) << "Reading two view geometries";
  colmap::TwoViewGeometry::Options two_view_geometry_options;
  two_view_geometry_options.ransac_options.max_error = 4.0;
  two_view_geometry_options.ransac_options.confidence = 0.999;
  two_view_geometry_options.ransac_options.min_num_trials = 100;
  two_view_geometry_options.ransac_options.max_num_trials = 10000;
  two_view_geometry_options.ransac_options.min_inlier_ratio = 0.25;

  // #pragma omp parallel for
  for (size_t i = 0; i < image_pairs.size(); i++) {
    const gopt::image_t image_id1 = image_pairs[i].first,
                        image_id2 = image_pairs[i].second;
    colmap::TwoViewGeometry& two_view_geometry = two_view_geometries[i];
    const auto& inlier_matches = two_view_geometry.inlier_matches;

    if (two_view_geometry.qvec.norm() < eps && two_view_geometry.tvec.norm() < eps) {
      // Re-estimate relative poses.
      const colmap::Camera& camera1 = reconstruction.Camera(image_id1);
      const colmap::Camera& camera2 = reconstruction.Camera(image_id2);
      const colmap::Image& image1 = reconstruction.Image(image_id1);
      const colmap::Image& image2 = reconstruction.Image(image_id2);
      
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

      two_view_geometry.EstimateWithRelativePose(
          camera1, points1, camera2, points2,
          inlier_matches, two_view_geometry_options);
    }

    gopt::graph::ViewEdge edge(
        image_id1 - 1, image_id2 - 1, two_view_geometry.inlier_matches.size());
    edge.rel_rotation = gopt::QuaternionToAngleAxis(two_view_geometry.qvec);
    edge.rel_translation = two_view_geometry.tvec;
    view_graph->AddEdge(edge);
  }

  database.ClearTwoViewGeometries();
  for (size_t i = 0; i < image_pairs.size(); i++) {
    const auto& two_view_geometry = two_view_geometries[i];
    const gopt::image_t image_id1 = image_pairs[i].first,
                        image_id2 = image_pairs[i].second;
    database.WriteTwoViewGeometry(image_id1, image_id2, two_view_geometry);
  }

  LOG(INFO) << "Nodes size: " << view_graph->GetNodes().size();
  LOG(INFO) << "image pairs: " << view_graph->GetEdgesNum();

  return view_graph->GetEdgesNum() > 0;
}
}  // namespace

void CastRotationEstimatorType(const std::string& rotation_estimator_type,
                               gopt::RotationEstimatorOptions* rotation_options) {
  if (rotation_estimator_type == "HYBRID") {
    rotation_options->estimator_type = gopt::GlobalRotationEstimatorType::HYBRID;
  } else if (rotation_estimator_type == "ROBUST_L1L2") {
    rotation_options->estimator_type = gopt::GlobalRotationEstimatorType::ROBUST_L1L2;
  } else {
    LOG(ERROR) << "Invalid rotation Estimator type! Valid Options: "
               << "[HYBRID, ROBUST_L1L2]";
  }
}

void CastPositionEstimatorType(const std::string& position_estimator_type,
                               gopt::PositionEstimatorOptions* position_options) {
  if (position_estimator_type == "LUD") {
    position_options->estimator_type = gopt::PositionEstimatorType::LUD;
  } else if (position_estimator_type == "LIGT") {
    position_options->estimator_type = gopt::PositionEstimatorType::LIGT;
  } else {
    LOG(ERROR) << "Invalid position estimator type! Valid Options: "
               << "[LUD, LIGT]";
  }
}

void RunIncrementalMapper(
    const colmap::IncrementalMapperOptions* options,
    const std::string& image_path,
    const std::string& database_path,
    const std::string& output_path) {
  colmap::ReconstructionManager reconstruction_manager;

  colmap::IncrementalMapperController mapper(
      options, image_path, database_path, &reconstruction_manager);

  mapper.Start();
  mapper.Wait();

  if (reconstruction_manager.Size() == 0) {
    LOG(ERROR) << "Failed to create ground truth model!";
    return;
  }

  const auto& reconstruction = reconstruction_manager.Get(0);
  gopt::graph::ViewGraph view_graph;
  LoadTwoViewGeometries(&view_graph, database_path, reconstruction);

  const std::string g2o_filename = colmap::JoinPaths(output_path,
      "VG_N" + std::to_string(view_graph.GetNodesNum()) +
      "_M" + std::to_string(view_graph.GetEdgesNum()) + ".g2o");
  view_graph.WriteG2OFile(g2o_filename);

  reconstruction.Write(output_path);
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  std::string output_path;
  std::string rotation_estimator_type = "ROBUST_L1L2";
  std::string position_estimator_type = "LUD";
  bool optimize_relative_translations = true;
  bool filter_relative_translations = true;
  bool final_global_bundle = true;
  bool run_incremental = false;

  colmap::OptionManager options;
  options.AddDatabaseOptions();
  options.AddImageOptions();
  options.AddRequiredOption("output_path", &output_path);
  options.AddDefaultOption("rotation_estimator_type",
                           &rotation_estimator_type);
  options.AddDefaultOption("position_estimator_type",
                           &position_estimator_type);
  options.AddDefaultOption("optimize_relative_translations",
                           &optimize_relative_translations);
  options.AddDefaultOption("filter_relative_translations",
                           &filter_relative_translations);
  options.AddDefaultOption("final_global_bundle", &final_global_bundle);
  options.AddDefaultOption("run_incremental", &run_incremental);

  options.Parse(argc, argv);

  // Firstly run incremental SfM in case that no intrinsics in the raw images,
  // which can cause essential matrix decomposition fails.
  if (run_incremental) {
    const std::string incremental_output_path = colmap::JoinPaths(
        output_path, "incremental");
    colmap::CreateDirIfNotExists(incremental_output_path);

    RunIncrementalMapper(
        options.mapper.get(), *options.image_path,
        *options.database_path, incremental_output_path);
  }

  colmap::ReconstructionManager reconstruction_manager;

  gopt::GlobalMapperOptions mapper_options;
  mapper_options.output_path = colmap::JoinPaths(output_path,
      "global_" + rotation_estimator_type + "+" + position_estimator_type);
  mapper_options.Mapper().optimize_relative_translations =
      optimize_relative_translations;
  mapper_options.Mapper().filter_relative_translations =
      filter_relative_translations;
  mapper_options.Mapper().final_global_bundle = final_global_bundle;

  CastRotationEstimatorType(rotation_estimator_type,
                            &mapper_options.RotationEstimator());
  CastPositionEstimatorType(position_estimator_type,
                            &mapper_options.PositionEstimator());

  colmap::CreateDirIfNotExists(output_path);
  colmap::CreateDirIfNotExists(mapper_options.output_path);

  gopt::GlobalMapperController mapper(&mapper_options, *options.image_path,
                                      *options.database_path,
                                      &reconstruction_manager);
  mapper.Start();
  mapper.Wait();
  
  if (reconstruction_manager.Size() == 0) {
    LOG(ERROR) << "ERROR: failed to create sparse model";
    return EXIT_FAILURE;
  }

  const colmap::Reconstruction& global_recon = reconstruction_manager.Get(0);
  global_recon.Write(mapper_options.output_path);

  return EXIT_SUCCESS;
}
