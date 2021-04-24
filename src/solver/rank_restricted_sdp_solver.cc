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

#include "solver/rank_restricted_sdp_solver.h"

#include <glog/logging.h>
#include <limits>

#include <Eigen/Eigenvalues>
#include <Eigen/QR>

namespace gopt {
namespace solver {

RankRestrictedSDPSolver::RankRestrictedSDPSolver(const size_t n,
                                                 const size_t block_dim)
    : RankRestrictedSDPSolver(n, block_dim, solver::SDPSolverOptions()) {}

RankRestrictedSDPSolver::RankRestrictedSDPSolver(
    const size_t n, const size_t block_dim,
    const solver::SDPSolverOptions& options)
    : BCMSDPSolver(n, block_dim, options),
      rank_(3) {
  Y_ = Eigen::MatrixXd::Zero(rank_, dim_ * n_);
}

void RankRestrictedSDPSolver::Solve(solver::Summary& summary) {
  Eigen::MatrixXd G = Eigen::MatrixXd::Zero(rank_, dim_ * n_);

#pragma omp parallel for num_threads(sdp_solver_options_.num_threads)
  // Compute inital G according to Equ.(3)
  for (size_t i = 0; i < n_; i++) {
    const std::vector<size_t>& adjs = adj_edges_[i];
    for (auto j : adjs) {
      G.block(0, i * dim_, rank_, dim_) +=
          Y_.block(0, j * dim_, rank_, dim_) *
          Q_.block(j * dim_, i * dim_, dim_, dim_);
    }
  }

  double prev_func_val = std::numeric_limits<double>::max();
  double cur_func_val = this->EvaluateFuncVal();
  double duration = 0.0;
  double error = 0.0;

  summary.begin_time = std::chrono::high_resolution_clock::now();
  while (summary.total_iterations_num < sdp_solver_options_.max_iterations) {
    if (sdp_solver_options_.verbose) {
      LogToStd(summary.total_iterations_num, prev_func_val, cur_func_val, error,
               duration);
    }

    if (IsConverge(prev_func_val, cur_func_val, sdp_solver_options_.tolerance, &error)) {
      break;
    }

    for (size_t i = 0; i < n_; i++) {
      const Eigen::MatrixXd G_block = G.block(0, i * dim_, rank_, dim_);
      Eigen::JacobiSVD<Eigen::MatrixXd> jacobi_svd(
          -G_block, Eigen::ComputeThinU | Eigen::ComputeThinV);

      const Eigen::MatrixXd prev_Y = Y_.block(0, i * dim_, rank_, dim_).eval();
      Y_.block(0, i * dim_, rank_, dim_) =
          jacobi_svd.matrixU() * jacobi_svd.matrixV().transpose();

      const std::vector<size_t>& adjs = adj_edges_[i];
#pragma omp parallel for num_threads(sdp_solver_options_.num_threads)
      for (size_t idx = 0; idx < adjs.size(); ++idx) {
        const size_t j = adjs[idx];
        G.block(0, j * dim_, rank_, dim_) +=
            (Y_.block(0, i * dim_, rank_, dim_) - prev_Y) *
            Q_.block(i * dim_, j * dim_, dim_, dim_);
      }
    }

    summary.total_iterations_num++;
    duration = summary.Duration();

    // Update function value
    prev_func_val = cur_func_val;
    cur_func_val = this->EvaluateFuncVal();
  }

  summary.total_iterations_num++;
  if (sdp_solver_options_.verbose) {
    LogToStd(summary.total_iterations_num, prev_func_val, cur_func_val, error,
             duration);
  }
}

Eigen::MatrixXd RankRestrictedSDPSolver::GetSolution() const {
  Eigen::MatrixXd solution = Eigen::MatrixXd(dim_, dim_ * n_);
  for (size_t i = 0; i < n_; i++) {
    solution.block(0, dim_ * i, dim_, dim_) =
        Y_.block(0, 0, rank_, dim_).transpose() *
        Y_.block(0, dim_ * i, rank_, dim_);
  }
  return solution;
}

const Eigen::MatrixXd& RankRestrictedSDPSolver::GetYStar() const {
  return Y_;
}

const Eigen::MatrixXd RankRestrictedSDPSolver::ComputeQYt(const Eigen::MatrixXd& Y) const {
  return Q_ * Y.transpose();
}

Eigen::MatrixXd RankRestrictedSDPSolver::ComputeLambdaMatrix() const {
  const Eigen::MatrixXd QYt = this->ComputeQYt(Y_);
  
  const size_t rank = Y_.rows();
  // \Lambda = \SymblockDiag(Q * Y^T * Y).
  Eigen::MatrixXd Lambda = Eigen::MatrixXd::Zero(dim_, n_ * dim_);

#pragma omp parallel for num_threads(sdp_solver_options_.num_threads)
  for (size_t i = 0; i < n_; ++i) {
    Eigen::MatrixXd P = QYt.block(i * dim_, 0, dim_, rank) *
      Y_.block(0, i * dim_, rank, dim_);
    Lambda.block(0, i * dim_, dim_, dim_) = 0.5 * (P + P.transpose());
  }

  return Lambda;
}

double RankRestrictedSDPSolver::EvaluateFuncVal() const {
  // tr(Q * Y^T * Y) = tr(Y * Q * Y^T)
  // return (Q_ * Y_.transpose() * Y_).trace();
  return (Y_ * (Q_ * Y_.transpose())).trace();
}

double RankRestrictedSDPSolver::EvaluateFuncVal(
    const Eigen::MatrixXd& Y) const {
  return (Y * (Q_ * Y.transpose())).trace();
}

void RankRestrictedSDPSolver::AugmentRank() {
  rank_++;
  Y_.conservativeResize(Y_.rows() + 1, Y_.cols());
  Y_.row(Y_.rows() - 1).setZero();
}

void RankRestrictedSDPSolver::SetOptimalY(const Eigen::MatrixXd& Y) {
  CHECK_EQ(Y.rows(), Y_.rows());
  CHECK_EQ(Y.cols(), Y_.cols());
  Y_ = Y;
}

size_t RankRestrictedSDPSolver::CurrentRank() const {
  return rank_;
}

Eigen::MatrixXd RankRestrictedSDPSolver::Project(const Eigen::MatrixXd& A) const {
  // We use a generalization of the well-known SVD-based projection for the
  // orthogonal and special orthogonal groups; see for example Proposition 7
  // in the paper "Projection-Like Retractions on Matrix Manifolds" by Absil
  // and Malick.

  Eigen::MatrixXd P(rank_, dim_ * n_);

#pragma omp parallel for num_threads(sdp_solver_options_.num_threads)
  for (size_t i = 0; i < n_; ++i) {
    // Compute the (thin) SVD of the ith block of A
    Eigen::JacobiSVD<Eigen::MatrixXd> SVD(A.block(0, i * dim_, rank_, dim_),
                                 Eigen::ComputeThinU | Eigen::ComputeThinV);

    // Set the ith block of P to the SVD-based projection of the ith block of A
    P.block(0, i * dim_, rank_, dim_) = SVD.matrixU() * SVD.matrixV().transpose();
  }
  return P;
}

Eigen::MatrixXd RankRestrictedSDPSolver::Retract(
    const Eigen::MatrixXd& Y, const Eigen::MatrixXd& V) const {
  // We use projection-based retraction, as described in "Projection-Like
  // Retractions on Matrix Manifolds" by Absil and Malick.
  return Project(Y + V);
}

Eigen::MatrixXd RankRestrictedSDPSolver::EuclideanGradient(
    const Eigen::MatrixXd& Y) const {
  return 2.0 * (Q_ * Y.transpose()).transpose();
}

Eigen::MatrixXd RankRestrictedSDPSolver::RiemannianGradient(
    const Eigen::MatrixXd& Y, const Eigen::MatrixXd& nablaF_Y) const {
  return TangentSpaceProjection(Y, nablaF_Y);
}

Eigen::MatrixXd RankRestrictedSDPSolver::RiemannianGradient(
    const Eigen::MatrixXd& Y) const {
  return TangentSpaceProjection(Y, EuclideanGradient(Y));
}

Eigen::MatrixXd RankRestrictedSDPSolver::TangentSpaceProjection(
    const Eigen::MatrixXd& Y, const Eigen::MatrixXd& dotY) const {
  return dotY - SymBlockDiagProduct(Y, Y, dotY);
}

Eigen::MatrixXd RankRestrictedSDPSolver::SymBlockDiagProduct(
    const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
    const Eigen::MatrixXd &C) const {
  // Preallocate result matrix
  Eigen::MatrixXd R(rank_, dim_ * n_);

#pragma omp parallel for num_threads(sdp_solver_options_.num_threads)
  for (size_t i = 0; i < n_; ++i) {
    // Compute block product Bi' * Ci.
    Eigen::MatrixXd P =
        B.block(0, i * dim_, rank_, dim_).transpose() *
            C.block(0, i * dim_, rank_, dim_);
    // Symmetrize this block.
    Eigen::MatrixXd S = 0.5 * (P + P.transpose());
    // Compute Ai * S and set corresponding block of R.
    R.block(0, i * dim_, rank_, dim_) = A.block(0, i * dim_, rank_, dim_) * S;
  }
  return R;
}

Eigen::MatrixXd RankRestrictedSDPSolver::Precondition(
    const Eigen::MatrixXd& Y, const Eigen::MatrixXd& dotY) const {
  switch (sdp_solver_options_.preconditioner_type) {
    case PreconditionerType::None:
      return dotY;
      break;
    case PreconditionerType::JACOBI:
      /* code */
      break;
    case PreconditionerType::INCOMPLETE_CHOLESKY:
      /* code */
      break;
    case PreconditionerType::REGULARIZED_CHOLESKY:
    default:
      break;
  }
}

}  // namespace solver
}  // namespace gopt
