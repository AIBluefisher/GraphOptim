#include "graph/view_graph.h"

#include <string>

#include <glog/logging.h>
#include <gflags/gflags.h>

#include "util/types.h"

DEFINE_string(g2o_filename, "", "The absolute path of g2o file");

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, false);

  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  if (argc < 2) {
    LOG(INFO) << "[Usage]: rotation_estimator --g2o_filename=g2o_filename";
    return 0;
  }

  std::string g2o_filename = FLAGS_g2o_filename;
  gopt::graph::ViewGraph view_graph;
  view_graph.ReadG2OFile(g2o_filename);
  
  gopt::RotationEstimatorOptions options;
  options.sdp_solver_options.verbose = true;
  // Set to 1e-6 for se-sync datasets.
  options.sdp_solver_options.tolerance = 1e-8;
  options.sdp_solver_options.max_iterations = 100;
  options.sdp_solver_options.riemannian_staircase_options.
      min_eigenvalue_nonnegativity_tolerance = 1e-2;
  std::unordered_map<gopt::image_t, Eigen::Vector3d> global_rotations;
  view_graph.RotationAveraging(options, &global_rotations);
}
