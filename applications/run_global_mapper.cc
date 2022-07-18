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
