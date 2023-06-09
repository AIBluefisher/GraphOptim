#include "geometry/align_point_clouds.h"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <glog/logging.h>

namespace gopt {

void AlignPointCloudsUmeyama(const std::vector<Eigen::Vector3d>& left,
                             const std::vector<Eigen::Vector3d>& right,
                             Eigen::Matrix3d* rotation,
                             Eigen::Vector3d* translation, double* scale) {
  std::vector<double> weights(left.size(), 1.0);
  AlignPointCloudsUmeyamaWithWeights(left, right, weights, rotation,
                                     translation, scale);
}

void AlignPointCloudsUmeyamaWithWeights(
    const std::vector<Eigen::Vector3d>& left,
    const std::vector<Eigen::Vector3d>& right,
    const std::vector<double>& weights, Eigen::Matrix3d* rotation,
    Eigen::Vector3d* translation, double* scale) {
  CHECK_EQ(left.size(), right.size());
  CHECK_EQ(left.size(), weights.size());
  CHECK_NOTNULL(rotation);
  CHECK_NOTNULL(translation);
  CHECK_NOTNULL(scale);

  // Fill outputs (useful when it fails)
  *scale = 1.0;
  *translation = Eigen::Vector3d::Zero();
  *rotation = Eigen::Matrix3d::Identity();

  const size_t num_points = left.size();
  Eigen::Map<const Eigen::Matrix<double, 3, Eigen::Dynamic> > left_points(
      left[0].data(), 3, num_points);
  Eigen::Map<const Eigen::Matrix<double, 3, Eigen::Dynamic> > right_points(
      right[0].data(), 3, num_points);

  Eigen::Vector3d left_centroid, right_centroid;
  left_centroid.setZero();
  right_centroid.setZero();
  double weights_sum = 0.0;
  for (size_t i = 0; i < num_points; i++) {
    CHECK_GE(weights[i], 0)
        << "The point weight must be greater or equal to zero.";
    weights_sum += weights[i];
    left_centroid += left[i] * weights[i];
    right_centroid += right[i] * weights[i];
  }
  // Check if the sum is valid
  CHECK_GT(weights_sum, 0) << "The sum of weights must be greater than zero.";

  left_centroid /= weights_sum;
  right_centroid /= weights_sum;

  double sigma = 0.0;
  for (size_t i = 0; i < num_points; i++) {
    sigma += (left[i] - left_centroid).squaredNorm() * weights[i];
  }
  sigma /= weights_sum;

  // Calculate cross correlation matrix based on the points shifted about the
  // centroid.
  Eigen::Matrix3d cross_correlation = Eigen::Matrix3d::Zero();
  for (size_t i = 0; i < num_points; i++) {
    cross_correlation += weights[i] * (left_points.col(i) - left_centroid) *
                         (right_points.col(i) - right_centroid).transpose();
  }
  cross_correlation /= weights_sum;

  // Compute SVD decomposition of the cross correlation.
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      cross_correlation.transpose(), Eigen::ComputeFullU | Eigen::ComputeFullV);

  Eigen::Matrix3d umatrix = svd.matrixU();
  Eigen::Matrix3d vtmatrix = svd.matrixV().transpose();
  const Eigen::Vector3d& singular_values = svd.singularValues();

  double det = umatrix.determinant() * vtmatrix.determinant();
  Eigen::Matrix3d s = Eigen::Matrix3d::Identity();
  s(2, 2) = det > 0 ? 1 : -1;

  *scale =
      (singular_values(0) + singular_values(1) + s(2, 2) * singular_values(2)) /
      sigma;
  *rotation = umatrix * s * vtmatrix;
  *translation = right_centroid - (*scale) * (*rotation) * left_centroid;
}

} // namespace gopt
