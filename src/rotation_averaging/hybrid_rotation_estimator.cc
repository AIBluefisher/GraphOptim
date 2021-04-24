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

#include "rotation_averaging/hybrid_rotation_estimator.h"

#include <ceres/rotation.h>
#include <glog/logging.h>
#include <omp.h>

#include <Eigen/Eigenvalues>
#include <Eigen/QR>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>

#include "Spectra/MatOp/SparseSymMatProd.h"
#include "Spectra/SymEigsSolver.h"

#include "rotation_averaging/internal/rotation_estimator_util.h"
#include "solver/bcm_sdp_solver.h"
#include "solver/rbr_sdp_solver.h"
#include "solver/rank_restricted_sdp_solver.h"
#include "solver/riemannian_staircase.h"

namespace gopt {

HybridRotationEstimator::HybridRotationEstimator(
    const int N, const int dim)
    : HybridRotationEstimator(N, dim, HybridRotationEstimatorOptions()) {}

HybridRotationEstimator::HybridRotationEstimator(
    const int N, const int dim,
    const HybridRotationEstimator::HybridRotationEstimatorOptions& options)
    : options_(options),
      images_num_(N),
      dim_(dim),
      ld_rotation_estimator_(new LagrangeDualRotationEstimator(
          N, dim, options.sdp_solver_options)),
      irls_rotation_refiner_(nullptr) {}

bool HybridRotationEstimator::EstimateRotations(
    const std::unordered_map<ImagePair, TwoViewGeometry>& view_pairs,
    std::unordered_map<image_t, Eigen::Vector3d>* global_rotations) {
  const int N = images_num_;

  CHECK_NOTNULL(global_rotations);
  CHECK_GT(N, 0);
  CHECK_EQ(N, (*global_rotations).size());
  CHECK_GT(view_pairs.size(), 0);

  irls_rotation_refiner_.reset(
      new IRLSRotationLocalRefiner(N, view_pairs.size(), options_.irls_options));
  
  internal::ViewIdToAscentIndex(*global_rotations, &view_id_to_index_);
  ld_rotation_estimator_->SetViewIdToIndex(view_id_to_index_);

  Eigen::SparseMatrix<double> sparse_matrix;
  internal::SetupLinearSystem(
      view_pairs, (*global_rotations).size(),
      view_id_to_index_, &sparse_matrix);
  irls_rotation_refiner_->SetViewIdToIndex(view_id_to_index_);
  irls_rotation_refiner_->SetSparseMatrix(sparse_matrix);

  // Estimate global rotations that resides within the cone of 
  // convergence for IRLS.
  LOG(INFO) << "Estimating Rotations Using LagrangeDual";
  ld_rotation_estimator_->EstimateRotations(view_pairs, global_rotations);

  // Refine the globally optimal result by IRLS.
  Eigen::VectorXd tangent_space_step;
  GlobalRotationsToTangentSpace(*global_rotations, &tangent_space_step);
  irls_rotation_refiner_->SetInitTangentSpaceStep(tangent_space_step);

  LOG(INFO) << "Refining Global Rotations";
  irls_rotation_refiner_->SolveIRLS(view_pairs, global_rotations);

  return true;
}

void HybridRotationEstimator::GlobalRotationsToTangentSpace(
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
