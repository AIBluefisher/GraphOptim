#include "geometry/rotation.h"

#include <Eigen/Geometry>

#include <gtest/gtest.h>
#include <iostream>

namespace mvgplus {

namespace {
Eigen::Matrix3d EulerAnglesToRotationMatrix(const double rx, const double ry,
                                            const double rz) {
  const Eigen::Matrix3d Rx =
      Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX()).toRotationMatrix();
  const Eigen::Matrix3d Ry =
      Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const Eigen::Matrix3d Rz =
      Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return Rz * Ry * Rx;
}

}  // namespace

TEST(TEST_ROTATION, TestQuaternionToRotationMatrix) {
  const double rx = 0;
  const double ry = 0;
  const double rz = 0.3;
  const Eigen::Matrix3d rot_mat0 = EulerAnglesToRotationMatrix(rx, ry, rz);
  const Eigen::Matrix3d rot_mat1 =
      QuaternionToRotationMatrix(RotationMatrixToQuaternion(rot_mat0));
  EXPECT_EQ(rot_mat0.isApprox(rot_mat1), true);
}

TEST(TEST_ROTATION, TestAngleAxisToRotationMatrix) {
  const double rx = 0;
  const double ry = 0;
  const double rz = 0.3;
  const Eigen::Matrix3d R1 = EulerAnglesToRotationMatrix(rx, ry, rz);
  std::cout << "R1: " << R1 << std::endl;
  const Eigen::Vector3d angle_axis = RotationMatrixToAngleAxis(R1);
  std::cout << "angle axis: " << angle_axis << std::endl;
  const Eigen::Matrix3d R2 =
      AngleAxisToRotationMatrix(angle_axis);

  EXPECT_EQ(R1.isApprox(R2), true);
}

TEST(TEST_ROTATION, TestQuaternionToAngleAxis) {
  const double rx = 0;
  const double ry = 0;
  const double rz = 0.3;
  const Eigen::Matrix3d R = EulerAnglesToRotationMatrix(rx, ry, rz);

  const Eigen::Vector4d quat1 = RotationMatrixToQuaternion(R);
  const Eigen::Vector4d quat2 =
      AngleAxisToQuaternion(QuaternionToAngleAxis(quat1));

  EXPECT_EQ(quat1.isApprox(quat2), true);
}

}  // mvgplus
