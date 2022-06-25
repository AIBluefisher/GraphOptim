// Copyright (c) 2021, Chenyu
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
  const Eigen::Vector3d angle_axis = RotationMatrixToAngleAxis(R1);
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
