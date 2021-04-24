// Copyright (C) 2014 The Regents of the University of California (Regents).
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials provided
//       with the distribution.
//
//     * Neither the name of The Regents or University of California nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Please contact the author of this library if you have any questions.
// Author: Chris Sweeney (cmsweeney@cs.ucsb.edu)

// BSD 3-Clause License

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

#include "geometry/rotation_utils.h"

#include <unordered_map>

#include <ceres/rotation.h>
#include <ceres/solver.h>
#include <ceres/problem.h>
#include <ceres/cost_function.h>
#include <ceres/autodiff_cost_function.h>
#include <glog/logging.h>

#include "util/map_util.h"

namespace {

// A cost function whose error is the difference in rotations after the current
// alignment is applied. That is,
//    error = unaligned_rotation * rotation_alignment - gt_rotation.
struct RotationAlignmentError {
  RotationAlignmentError(const Eigen::Vector3d& gt_rotation,
                         const Eigen::Vector3d& unaligned_rotation)
      : gt_rotation_(gt_rotation) {
    // Convert the unaligned rotation to rotation matrix.
    Eigen::Matrix3d unaligned_rotation_mat;
    ceres::AngleAxisToRotationMatrix(
        unaligned_rotation.data(),
        ceres::ColumnMajorAdapter3x3(unaligned_rotation_mat_.data()));
  }

  // Compute the alignment error of the two rotations after applying the
  // rotation transformation.
  template <typename T>
  bool operator()(const T* rotation, T* residuals) const {
    // Convert the rotation transformation to a matrix.
    Eigen::Matrix<T, 3, 3> rotation_mat;
    ceres::AngleAxisToRotationMatrix(
        rotation, ceres::ColumnMajorAdapter3x3(rotation_mat.data()));

    // Apply the rotation transformation.
    const Eigen::Matrix<T, 3, 3> aligned_rotation_mat =
        unaligned_rotation_mat_.cast<T>() * rotation_mat;

    // Convert back to angle axis.
    Eigen::Matrix<T, 3, 1> aligned_rotation;
    ceres::RotationMatrixToAngleAxis(
        ceres::ColumnMajorAdapter3x3(aligned_rotation_mat.data()),
        aligned_rotation.data());

    // Compute the error of the aligned rotation to the gt_rotation.
    residuals[0] = gt_rotation_[0] - aligned_rotation[0];
    residuals[1] = gt_rotation_[1] - aligned_rotation[1];
    residuals[2] = gt_rotation_[2] - aligned_rotation[2];

    return true;
  }

  static ceres::CostFunction* Create(
      const Eigen::Vector3d& gt_rotation,
      const Eigen::Vector3d& unaligned_rotation) {
    return new ceres::AutoDiffCostFunction<RotationAlignmentError, 3, 3>(
        new RotationAlignmentError(gt_rotation, unaligned_rotation));
  }

  Eigen::Vector3d gt_rotation_;
  Eigen::Matrix3d unaligned_rotation_mat_;
};

// Apply the rotation alignment to all rotations in the vector.
void ApplyRotationTransformation(const Eigen::Vector3d& rotation_alignment,
                                 std::vector<Eigen::Vector3d>* rotation) {
  Eigen::Matrix3d rotation_alignment_mat;
  ceres::AngleAxisToRotationMatrix(
      rotation_alignment.data(),
      ceres::ColumnMajorAdapter3x3(rotation_alignment_mat.data()));

  for (size_t i = 0; i < rotation->size(); i++) {
    // Convert the current rotation to a rotation matrix.
    Eigen::Matrix3d rotation_mat;
    ceres::AngleAxisToRotationMatrix(
        rotation->at(i).data(),
        ceres::ColumnMajorAdapter3x3(rotation_mat.data()));

    // Apply the rotation transformation.
    const Eigen::Matrix3d aligned_rotation =
        rotation_mat * rotation_alignment_mat;

    // Convert back to angle axis.
    ceres::RotationMatrixToAngleAxis(
        ceres::ColumnMajorAdapter3x3(aligned_rotation.data()),
        rotation->at(i).data());
  }
}

}  // namespace

namespace gopt {
namespace geometry {

Eigen::MatrixXd ProjectToSOd(const Eigen::MatrixXd &M) {
  // Compute the SVD of M.
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      M, Eigen::ComputeFullU | Eigen::ComputeFullV);

  double detU = svd.matrixU().determinant();
  double detV = svd.matrixV().determinant();

  if (detU * detV > 0) {
    return svd.matrixU() * svd.matrixV().transpose();
  } else {
    Eigen::MatrixXd U_prime = svd.matrixU();
    U_prime.col(U_prime.cols() - 1) *= -1;
    return U_prime * svd.matrixV().transpose();
  }
}

Eigen::Vector3d MultiplyRotations(const Eigen::Vector3d& rotation1,
                                  const Eigen::Vector3d& rotation2) {
  Eigen::Matrix3d rotation1_mat, rotation2_mat;
  ceres::AngleAxisToRotationMatrix(rotation1.data(), rotation1_mat.data());
  ceres::AngleAxisToRotationMatrix(rotation2.data(), rotation2_mat.data());

  const Eigen::Matrix3d rotation = rotation1_mat * rotation2_mat;
  Eigen::Vector3d rotation_aa;
  ceres::RotationMatrixToAngleAxis(rotation.data(), rotation_aa.data());
  return rotation_aa;
}

// Computes R_ij = R_j * R_i^t.
Eigen::Vector3d RelativeRotationFromTwoRotations(
    const Eigen::Vector3d& rotation1, const Eigen::Vector3d& rotation2,
    const double noise) {
  const Eigen::Matrix3d noisy_rotation =
      Eigen::AngleAxisd(DegToRad(noise), rng.RandVector3d().normalized())
          .toRotationMatrix();

  Eigen::Matrix3d rotation_matrix1, rotation_matrix2;
  ceres::AngleAxisToRotationMatrix(rotation1.data(), rotation_matrix1.data());
  ceres::AngleAxisToRotationMatrix(rotation2.data(), rotation_matrix2.data());

  const Eigen::AngleAxisd relative_rotation(noisy_rotation * rotation_matrix2 *
                                            rotation_matrix1.transpose());
  return relative_rotation.angle() * relative_rotation.axis();
}

// Computes R_ij = R_j * R_i^t.
Eigen::Vector3d RelativeRotationFromTwoRotations(
    const Eigen::Vector3d& rotation1, const Eigen::Vector3d& rotation2) {
  Eigen::Matrix3d rotation_matrix1, rotation_matrix2;
  ceres::AngleAxisToRotationMatrix(rotation1.data(), rotation_matrix1.data());
  ceres::AngleAxisToRotationMatrix(rotation2.data(), rotation_matrix2.data());

  const Eigen::AngleAxisd relative_rotation(rotation_matrix2 *
                                            rotation_matrix1.transpose());
  return relative_rotation.angle() * relative_rotation.axis();
}

Eigen::Vector3d ApplyRelativeRotation(
    const Eigen::Vector3d& rotation1,
    const Eigen::Vector3d& relative_rotation) {
  Eigen::Vector3d rotation2;
  Eigen::Matrix3d rotation1_matrix, relative_rotation_matrix;
  ceres::AngleAxisToRotationMatrix(
      rotation1.data(), ceres::ColumnMajorAdapter3x3(rotation1_matrix.data()));
  ceres::AngleAxisToRotationMatrix(
      relative_rotation.data(),
      ceres::ColumnMajorAdapter3x3(relative_rotation_matrix.data()));

  const Eigen::Matrix3d rotation2_matrix =
      relative_rotation_matrix * rotation1_matrix;
  ceres::RotationMatrixToAngleAxis(
      ceres::ColumnMajorAdapter3x3(rotation2_matrix.data()), rotation2.data());
  return rotation2;
}

// We solve a nonlinear least squares system to determine a rotation R that will
// align the rotation to the gt_rotation such that rotation * R = gt_rotation.
// This could potentially be set up as a linear system, however, that does not
// guarantee that R will remain a valid rotation. Instead, we simply use a
// nonlinear system to ensure that R is a valid rotation.
void AlignRotations(const std::vector<Eigen::Vector3d>& gt_rotation,
                    std::vector<Eigen::Vector3d>* rotation) {
  CHECK_EQ(gt_rotation.size(), rotation->size());

  Eigen::Vector3d rotation_alignment = Eigen::Vector3d::Zero();

  // Set up the nonlinear system and adds all residuals.
  ceres::Problem problem;
  for (size_t i = 0; i < gt_rotation.size(); i++) {
    problem.AddResidualBlock(
        RotationAlignmentError::Create(gt_rotation[i], rotation->at(i)), NULL,
        rotation_alignment.data());
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.function_tolerance = 0.0;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  VLOG(2) << summary.FullReport();

  // Apply the solved rotation transformation to the rotations.
  ApplyRotationTransformation(rotation_alignment, rotation);
}

void AlignOrientations(
    const std::unordered_map<image_t, Eigen::Vector3d>& gt_rotations,
    std::unordered_map<image_t, Eigen::Vector3d>* rotations) {
  // Collect all rotations into a vector.
  std::vector<Eigen::Vector3d> gt_rot, rot;
  std::unordered_map<int, int> index_to_view_id;
  int current_index = 0;
  for (const auto& gt_rotation : gt_rotations) {
    gt_rot.emplace_back(gt_rotation.second);
    rot.emplace_back(FindOrDie(*rotations, gt_rotation.first));

    index_to_view_id[current_index] = gt_rotation.first;
    ++current_index;
  }

  AlignRotations(gt_rot, &rot);

  for (unsigned int i = 0; i < rot.size(); i++) {
    const image_t view_id = FindOrDie(index_to_view_id, i);
    (*rotations)[view_id] = rot[i];
  }
}

double RadToDeg(double angle_radians) {
  return angle_radians * kRadToDeg;
}

double DegToRad(double angle_degrees) {
  return angle_degrees * kDegToRad;
}

double Clamp(const double val, const double min, const double max) {
  return std::max(min, std::min(val, max));
}

Eigen::Vector3d RelativeTranslationFromTwoPositions(
    const Eigen::Vector3d& position1, const Eigen::Vector3d& position2,
    const Eigen::Vector3d& rotation1) {
  Eigen::Matrix3d rotation_matrix1;
  ceres::AngleAxisToRotationMatrix(rotation1.data(), rotation_matrix1.data());
  const Eigen::Vector3d relative_translation =
      rotation_matrix1 * (position2 - position1).normalized();
  return relative_translation;
}

}  // namespace geometry
}  // namespace gopt