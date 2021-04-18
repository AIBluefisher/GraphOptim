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
    LOG(INFO) << "[Usage]: position_estimator --g2o_filename=g2o_filename";
    return 0;
  }

  std::string g2o_filename = FLAGS_g2o_filename;
  gopt::graph::ViewGraph view_graph;
  view_graph.ReadG2OFile(g2o_filename);
  
  gopt::PositionEstimatorOptions options;

  std::unordered_map<gopt::image_t, Eigen::Vector3d> global_positions;
  view_graph.TranslationAveraging(options, &global_positions);
}
