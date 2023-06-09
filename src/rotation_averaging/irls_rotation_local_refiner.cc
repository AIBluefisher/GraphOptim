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

#include "rotation_averaging/irls_rotation_local_refiner.h"

#include <iomanip>
#include <iostream>
#include <glog/logging.h>

#include "rotation_averaging/rotation_estimator.h"
#include "rotation_averaging/internal/rotation_estimator_util.h"
#include "geometry/rotation_utils.h"
#include "math/sparse_cholesky_llt.h"
#include "utils/map_util.h"
#include "utils/types.h"
#include "utils/timer.h"

namespace gopt {

IRLSRotationLocalRefiner::IRLSRotationLocalRefiner(
    const int num_orientations, const int num_edges,
    const IRLSRefinerOptions& options)
  : options_(options) {
  // The rotation change is one less than the number of global rotations because
  // we keep one rotation constant.
  tangent_space_step_.resize((num_orientations - 1) * 3);
  tangent_space_residual_.resize(num_edges * 3);
}

void IRLSRotationLocalRefiner::SetInitTangentSpaceStep(
    const Eigen::VectorXd& tangent_space_step) {
  tangent_space_step_ = tangent_space_step;
}

void IRLSRotationLocalRefiner::SetViewIdToIndex(
    const std::unordered_map<image_t, int>& view_id_to_index) {
  view_id_to_index_ = view_id_to_index;
}

void IRLSRotationLocalRefiner::SetSparseMatrix(
    const Eigen::SparseMatrix<double>& sparse_matrix) {
  sparse_matrix_ = sparse_matrix;
}

bool IRLSRotationLocalRefiner::SolveIRLS(
    const std::unordered_map<ImagePair, TwoViewGeometry>& relative_rotations,
    std::unordered_map<image_t, Eigen::Vector3d>* global_rotations) {
  const int num_edges = relative_rotations.size();

  CHECK_NOTNULL(global_rotations);
  CHECK_GT(global_rotations->size(), 0);
  CHECK_GT(num_edges, 0);

  if (view_id_to_index_.empty()) {
    internal::ViewIdToAscentIndex(*global_rotations, &view_id_to_index_);
  }

  if (sparse_matrix_.rows() == 0) {
    internal::SetupLinearSystem(
        relative_rotations, (*global_rotations).size(),
        view_id_to_index_, &sparse_matrix_);
  }

  // Set up the linear solver and analyze the sparsity pattern of the
  // system. Since the sparsity pattern will not change with each linear solve
  // this can help speed up the solution time.
  SparseCholeskyLLt linear_solver;
  linear_solver.AnalyzePattern(sparse_matrix_.transpose() * sparse_matrix_);
  if (linear_solver.Info() != Eigen::Success) {
    LOG(ERROR) << "Cholesky decomposition failed.";
    return false;
  }

  if (options_.verbose) {
    LOG(INFO) << std::setw(12) << std::setfill(' ') << "Iter "
              << std::setw(16) << std::setfill(' ') << "SqError "
              << std::setw(16) << std::setfill(' ') << "Delta ";
  }

  ComputeResiduals(relative_rotations, global_rotations);

  Eigen::ArrayXd weights(num_edges * 3);
  Eigen::SparseMatrix<double> at_weight;
  Timer timer;
  timer.Start();
  for (int i = 0; i < options_.max_num_irls_iterations; i++) {
    // Compute the Huber-like weights for each error term.
    const double& sigma = options_.irls_loss_parameter_sigma;
#pragma omp parallel for num_threads(options_.num_threads)
    for (int k = 0; k < num_edges; ++k) {
      double e_sq = tangent_space_residual_.segment<3>(3 * k).squaredNorm();
      double tmp = e_sq + sigma * sigma;
      double w = sigma / (tmp * tmp);
      weights.segment<3>(3 * k).setConstant(w);
    }

    // Update the factorization for the weighted values.
    at_weight = sparse_matrix_.transpose() * weights.matrix().asDiagonal();
    linear_solver.Factorize(at_weight * sparse_matrix_);
    if (linear_solver.Info() != Eigen::Success) {
      LOG(ERROR) << "Failed to factorize the least squares system.";
      return false;
    }

    // Solve the least squares problem.
    tangent_space_step_ =
        linear_solver.Solve(at_weight * tangent_space_residual_);
    if (linear_solver.Info() != Eigen::Success) {
      LOG(ERROR) << "Failed to solve the least squares system.";
      return false;
    }

    UpdateGlobalRotations(global_rotations);
    ComputeResiduals(relative_rotations, global_rotations);
    const double avg_step_size = ComputeAverageStepSize();

    if (options_.verbose) {
      LOG(INFO) << std::setw(12) << std::setfill(' ') << i
                << std::setw(16) << std::setfill(' ')
                << tangent_space_residual_.squaredNorm()
                << std::setw(16) << std::setfill(' ') << avg_step_size;
    }

    if (avg_step_size < options_.irls_step_convergence_threshold &&
        options_.verbose) {
      LOG(INFO) << "IRLS Converged in " << i + 1 << " iterations.";
      break;
    }
  }
  timer.Pause();

  if (options_.verbose) {
    LOG(INFO) << "Total time [IRLS]: "
              << timer.ElapsedMicroSeconds() * 1e-3 << " ms.";
  }
  return true;
}

void IRLSRotationLocalRefiner::UpdateGlobalRotations(
    std::unordered_map<image_t, Eigen::Vector3d>* global_rotations) {
  for (auto& rotation : *global_rotations) {
    const int view_index = FindOrDie(view_id_to_index_, rotation.first) - 1;
    if (view_index == kConstantRotationIndex) {
      continue;
    }

    // Apply the rotation change to the global orientation.
    const Eigen::Vector3d& rotation_change =
        tangent_space_step_.segment<3>(3 * view_index);
    rotation.second = geometry::MultiplyRotations(rotation.second, rotation_change);
  }
}

void IRLSRotationLocalRefiner::ComputeResiduals(
    const std::unordered_map<ImagePair, TwoViewGeometry>& relative_rotations,
    std::unordered_map<image_t, Eigen::Vector3d>* global_rotations) {
  int rotation_error_index = 0;

  for (const auto& relative_rotation : relative_rotations) {
    const Eigen::Vector3d& relative_rotation_aa = relative_rotation.second.rel_rotation;
    const Eigen::Vector3d& rotation1 =
        FindOrDie(*global_rotations, relative_rotation.first.first);
    const Eigen::Vector3d& rotation2 =
        FindOrDie(*global_rotations, relative_rotation.first.second);

    // Compute the relative rotation error as:
    //   R_err = R2^t * R_12 * R1.
    tangent_space_residual_.segment<3>(3 * rotation_error_index) =
        geometry::MultiplyRotations(-rotation2,
            geometry::MultiplyRotations(relative_rotation_aa, rotation1));
    ++rotation_error_index;
  }
}

double IRLSRotationLocalRefiner::ComputeAverageStepSize() {
  // compute the average step size of the update in tangent_space_step_
  const int num_vertices = tangent_space_step_.size() / 3;
  double delta_V = 0;
  for (int k = 0; k < num_vertices; ++k) {
    delta_V += tangent_space_step_.segment<3>(3 * k).norm();
  }
  return delta_V / num_vertices;
}

}  // namespace gopt
