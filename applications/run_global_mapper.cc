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
#include <colmap/base/image.h>
#include <colmap/base/reconstruction_manager.h>

#include <gopt/graph/view_graph.h>

#include "global_mapper_controller.h"

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

  options.Parse(argc, argv);

  colmap::ReconstructionManager reconstruction_manager;

  gopt::GlobalMapperOptions mapper_options;
  mapper_options.output_path = output_path;
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

  gopt::GlobalMapperController mapper(&mapper_options, *options.image_path,
                                      *options.database_path,
                                      &reconstruction_manager);
  mapper.Start();
  mapper.Wait();
  
  if (reconstruction_manager.Size() == 0) {
    LOG(ERROR) << "ERROR: failed to create sparse model";
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
