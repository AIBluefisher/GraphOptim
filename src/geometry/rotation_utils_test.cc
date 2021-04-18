#include "geometry/rotation_utils.h"

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <glog/logging.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "gtest/gtest.h"
#include "math/util.h"
#include "util/random.h"

namespace gopt {

namespace {

RandomNumberGenerator rng(49);

void ApplyRotation(const Eigen::Matrix3d& rotation_transformation,
                   const double noise, Eigen::Vector3d* rotation) {
  const Eigen::Matrix3d noisy_rotation =
      Eigen::AngleAxisd(DegToRad(noise), rng.RandVector3d().normalized())
          .toRotationMatrix();

  // Apply the transformation to the rotation.
  Eigen::Matrix3d rotation_mat;
  ceres::AngleAxisToRotationMatrix(
      rotation->data(), ceres::ColumnMajorAdapter3x3(rotation_mat.data()));
  const Eigen::Matrix3d transformed_rotation =
      rotation_mat * (noisy_rotation * rotation_transformation);

  // Convert back to angle axis.
  ceres::RotationMatrixToAngleAxis(
      ceres::ColumnMajorAdapter3x3(transformed_rotation.data()),
      rotation->data());
}

void TestAlignRotations(const int num_views, const double noise_degrees,
                        const double tolerance) {
  std::vector<Eigen::Vector3d> gt_rotations(num_views);
  std::vector<Eigen::Vector3d> rotations(num_views);

  Eigen::Matrix3d rotation_transformation =
      Eigen::AngleAxisd(15.0, rng.RandVector3d().normalized())
          .toRotationMatrix();
  for (int i = 0; i < num_views; i++) {
    gt_rotations[i] = rng.RandVector3d();
    rotations[i] = gt_rotations[i];
    ApplyRotation(rotation_transformation, noise_degrees, &rotations[i]);
  }

  AlignRotations(gt_rotations, &rotations);

  for (int i = 0; i < num_views; i++) {
    EXPECT_LT((gt_rotations[i] - rotations[i]).norm(), tolerance);
  }
}

}  // namespace

TEST(AlignRotations, NoNoise) {
  static const int kNumViews = 20;
  static const double kTolerance = 1e-8;
  static const double kNoise = 0.0;
  TestAlignRotations(kNumViews, kNoise, kTolerance);
}

TEST(AlignRotations, Noise) {
  static const int kNumViews = 20;
  static const double kTolerance = 5e-2;
  static const double kNoise = 1.0;
  TestAlignRotations(kNumViews, kNoise, kTolerance);
}

}  // namespace gopt