#include "graph/view_graph.h"

#include <gtest/gtest.h>

#include "geometry/rotation_utils.h"
#include "test/view_graph_generator.h"

namespace gopt {
namespace graph {

TEST(VIEW_GRAPH_TEST, TEST_INITIALIZE_GLOBAL_ROTATIONS_FROM_MST) {
  const size_t num_nodes = 10;
  const double completeness = 0.5;
  const double sigma = 10;
  const double outlier_ratio = 0;
  
  ViewGraphGenerator::ViewGraphGeneratorOptions options;
  ViewGraphGenerator generator(options);
  std::unordered_map<gopt::image_t, Eigen::Vector3d> gt_rotations;

  ViewGraph view_graph = generator.GenerateRandomGraph(
    num_nodes, completeness, sigma, outlier_ratio, 0, 0,
    &gt_rotations, nullptr);

  // Change the gauge freedom of ground truth rotations.
  const Eigen::Vector3d gt_angle_axis0 = gt_rotations.at(0);
  for (image_t i = 0; i < num_nodes; i++) {
    const Eigen::Vector3d angle_axis = gt_rotations[i];
    gt_rotations[i] = geometry::RelativeRotationFromTwoRotations(
      gt_angle_axis0, angle_axis);
  }
  
  std::unordered_map<gopt::image_t, Eigen::Vector3d> mst_rotations;
  view_graph.InitializeGlobalRotationsFromMST(&mst_rotations);
  // Change the gauge freedom of ground truth rotations.
  const Eigen::Vector3d mst_angle_axis0 = mst_rotations.at(0);
  for (image_t i = 0; i < num_nodes; i++) {
    const Eigen::Vector3d angle_axis = mst_rotations[i];
    mst_rotations[i] = geometry::RelativeRotationFromTwoRotations(
      mst_angle_axis0, angle_axis);
  }

  // Align the rotations and measure the error.
  geometry::AlignOrientations(gt_rotations, &mst_rotations);
  double sum_angular_error = 0.0;
  std::vector<double> angular_errors;

  for (size_t i = 0; i < gt_rotations.size(); i++) {
    const auto& rotation = gt_rotations.at(i);
    const Eigen::Vector3d& mst_rotation = FindOrDie(mst_rotations, i);
    const Eigen::Vector3d relative_rotation =
        geometry::RelativeRotationFromTwoRotations(mst_rotation,
                                                   rotation, 0.0);
    const double angular_error = geometry::RadToDeg(relative_rotation.norm());

    sum_angular_error += angular_error;
    angular_errors.push_back(angular_error);
  }

  std::sort(angular_errors.begin(), angular_errors.end());
  const double mean_angular_error = sum_angular_error / angular_errors.size();
  const double median_angular_error = angular_errors[num_nodes / 2];
  
  CHECK_LE(mean_angular_error, 15);
  CHECK_LE(median_angular_error, 15);

  std::cout << "\n";
  LOG(INFO) << "Mean Angular Residual (deg): " << mean_angular_error;
  LOG(INFO) << "Median Angular Residual (deg): " << median_angular_error;
  LOG(INFO) << "Max Angular Residual (deg): " << angular_errors[num_nodes - 1];
  LOG(INFO) << "Min Angular Residual (deg): " << angular_errors[0];
}

}  // namespace graph
}  // namespace gopt
