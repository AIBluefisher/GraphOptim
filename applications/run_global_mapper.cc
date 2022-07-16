#include <cstdlib>
#include <iostream>

#include <colmap/util/misc.h>
#include <colmap/util/option_manager.h>
#include <colmap/util/string.h>
#include <colmap/base/image.h>
#include <colmap/base/reconstruction_manager.h>

#include <gopt/graph/view_graph.h>

#include "global_mapper_controller.h"

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  std::string input_path;
  std::string output_path;

  colmap::OptionManager options;
  options.AddDatabaseOptions();
  options.AddImageOptions();
  options.AddRequiredOption("output_path", &output_path);
  options.Parse(argc, argv);

  colmap::ReconstructionManager reconstruction_manager;

  gopt::GlobalMapperOptions mapper_options;
  mapper_options.output_path = output_path;
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
