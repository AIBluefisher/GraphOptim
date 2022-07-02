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

#include "rotation_averaging/robust_l1l2_rotation_estimator.h"

#include <ceres/rotation.h>
#include <glog/logging.h>
#include <omp.h>

#include <Eigen/Eigenvalues>
#include <Eigen/QR>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>

#include "rotation_averaging/internal/rotation_estimator_util.h"

namespace gopt {

RobustL1L2RotationEstimator::RobustL1L2RotationEstimator(
    const RobustL1L2RotationEstimator::RobustL1L2RotationEstimatorOptions& options)
    : options_(options),
      l1_rotation_estimator_(nullptr),
      irls_rotation_refiner_(nullptr) {}

bool RobustL1L2RotationEstimator::EstimateRotations(
    const std::unordered_map<ImagePair, TwoViewGeometry>& view_pairs,
    std::unordered_map<image_t, Eigen::Vector3d>* global_rotations) {
  const int N = (*global_rotations).size();

  CHECK_NOTNULL(global_rotations);
  CHECK_GT(N, 0);
  CHECK_GT(view_pairs.size(), 0);

  Eigen::SparseMatrix<double> sparse_matrix;
  // LOG(INFO) << "Setup linear system";
  internal::ViewIdToAscentIndex(*global_rotations, &view_id_to_index_);
  internal::SetupLinearSystem(
      view_pairs, (*global_rotations).size(),
      view_id_to_index_, &sparse_matrix);

  l1_rotation_estimator_.reset(
     new L1RotationGlobalEstimator(N, view_pairs.size(), options_.l1_options));
  irls_rotation_refiner_.reset(
      new IRLSRotationLocalRefiner(N, view_pairs.size(), options_.irls_options));
  
  l1_rotation_estimator_->SetViewIdToIndex(view_id_to_index_);
  l1_rotation_estimator_->SetSparseMatrix(sparse_matrix);

  irls_rotation_refiner_->SetViewIdToIndex(view_id_to_index_);
  irls_rotation_refiner_->SetSparseMatrix(sparse_matrix);

  // Estimate global rotations that resides within the cone of 
  // convergence for IRLS.
  // LOG(INFO) << "Estimating Rotations Using L1Solver";
  l1_rotation_estimator_->SolveL1Regression(view_pairs, global_rotations);

  // Refine the globally optimal result by IRLS.
  Eigen::VectorXd tangent_space_step;
  GlobalRotationsToTangentSpace(*global_rotations, &tangent_space_step);
  irls_rotation_refiner_->SetInitTangentSpaceStep(tangent_space_step);

  // LOG(INFO) << "Refining Global Rotations";
  irls_rotation_refiner_->SolveIRLS(view_pairs, global_rotations);

  return true;
}

void RobustL1L2RotationEstimator::GlobalRotationsToTangentSpace(
    const std::unordered_map<image_t, Eigen::Vector3d>& global_rotations,
    Eigen::VectorXd* tangent_space_step) {
  (*tangent_space_step).resize((global_rotations.size() - 1) * 3);

  for (const auto& rotation : global_rotations) {
    const int view_index = FindOrDie(view_id_to_index_, rotation.first) - 1;

    if (view_index == IRLSRotationLocalRefiner::kConstantRotationIndex) {
      continue;
    }

    (*tangent_space_step).segment<3>(3 * view_index) = rotation.second;
  }
}

}  // namespace gopt
