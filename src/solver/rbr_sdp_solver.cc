#include "rbr_sdp_solver.h"

#include <ceres/rotation.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>

#include <Eigen/Eigenvalues>
#include <Eigen/QR>

#include "bcm_sdp_solver.h"
#include "math/matrix_square_root.h"

namespace gopt {
namespace solver {

RBRSDPSolver::RBRSDPSolver(const size_t n, const size_t block_dim)
    : RBRSDPSolver(n, block_dim, solver::SDPSolverOptions()) {}

RBRSDPSolver::RBRSDPSolver(
    const size_t n, const size_t block_dim,
    const solver::SDPSolverOptions& options)
    : BCMSDPSolver(n, block_dim, options){
  X_ = Eigen::MatrixXd::Identity(dim_ * n, dim_ * n);
}

void RBRSDPSolver::Solve(solver::Summary& summary) {
  double prev_func_val = std::numeric_limits<double>::max();
  double cur_func_val = this->EvaluateFuncVal();
  double duration = 0.0;
  double error = 0.0;

  summary.begin_time = std::chrono::high_resolution_clock::now();
  while (summary.total_iterations_num < sdp_solver_options_.max_iterations) {
    if (sdp_solver_options_.verbose) {
      this->LogToStd(summary.total_iterations_num, prev_func_val, cur_func_val,
                     error, duration);
    }

    if (IsConverge(prev_func_val, cur_func_val, sdp_solver_options_.tolerance, &error)) {
      break;
    }

    // convergence rate? Take it for consideration.
    for (size_t k = 0; k < n_; k++) {
      // Eliminating the k-th row and column from Y to form Bk
      Eigen::MatrixXd B = Eigen::MatrixXd::Zero(3 * (n_ - 1), 3 * (n_ - 1));
      this->ReformingB(k, B);

      // Eliminating the k-th column and all but the k-th row from R to form Wk
      Eigen::MatrixXd W = Eigen::MatrixXd::Zero(3 * (n_ - 1), 3);
      this->ReformingW(k, W);

      Eigen::MatrixXd B_multi_W = B * W;
      Eigen::MatrixXd WtBW = W.transpose() * B_multi_W;

      // FIXME: (chenyu) Solving matrix square root with
      // SVD and LDL^T would generate different result
      Eigen::MatrixXd WtBW_sqrt = MatrixSquareRoot(WtBW);
      // Eigen::MatrixXd WtBW_sqrt =
      // MatrixSquareRootForSemidefinitePositiveMat(WtBW);

      // FIXME: (chenyu) Eigen 3.3.0 is required for the use of
      // CompleteOrthogonalDecomposition<>
      // Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cqr(WtBW_sqrt);
      // Eigen::Matrix3d moore_penrose_pseinv = cqr.pseudoInverse();
      Eigen::Matrix3d moore_penrose_pseinv = WtBW_sqrt.inverse();

      // compute S by fixing the error of Equ.(47) in Erikson's paper
      Eigen::MatrixXd S = -B_multi_W * moore_penrose_pseinv;

      // reordering X
      this->ReorderingUnknown(k, B, S);
    }

    summary.total_iterations_num++;
    duration = summary.Duration();

    // Update function value
    prev_func_val = cur_func_val;
    cur_func_val = this->EvaluateFuncVal();
  }

  summary.total_iterations_num++;
  if (sdp_solver_options_.verbose) {
    this->LogToStd(summary.total_iterations_num, prev_func_val, cur_func_val,
                   error, duration);
  }
}

double RBRSDPSolver::EvaluateFuncVal() const {
  return EvaluateFuncVal(X_);
}

double RBRSDPSolver::EvaluateFuncVal(const Eigen::MatrixXd& Y) const {
  return (Q_ * Y).trace();
}

void RBRSDPSolver::ReformingB(const size_t k, Eigen::MatrixXd& B) {
  size_t r = 0, c = 0;  // the row and column index of matrix B

  for (size_t i = 0; i < n_; i++) {
    if (i == k) continue;
    c = 0;
    for (size_t j = 0; j < n_; j++) {
      if (j == k) continue;

      B.block(3 * r, 3 * c, 3, 3) = X_.block(3 * i, 3 * j, 3, 3);
      c++;
    }
    r++;
  }
}

void RBRSDPSolver::ReformingW(const size_t k, Eigen::MatrixXd& W) {
  size_t r = 0;  // row index of matrix W

  for (size_t i = 0; i < n_; i++) {
    if (i == k) continue;

    W.block(3 * r, 0, 3, 3) = Q_.block(3 * i, 3 * k, 3, 3);
    r++;
  }
}

void RBRSDPSolver::ReorderingUnknown(const size_t k, const Eigen::MatrixXd& B,
                                     const Eigen::MatrixXd& S) {
  // Reordering X according to [Algorithm 1] in paper:
  // - Z. Wen, D. Goldfarb, S. Ma, and K. Scheinberg.
  //   Row by row methods for semidefinite programming.
  //   Technical report, Columbia University, 2009. 7

  // Update X(k, k)
  X_.block(3 * k, 3 * k, 3, 3) = Eigen::Matrix3d::Identity();

  // Update the k-th column of Y, except Y(k, k)
  size_t j = 0;  // the j-th block of S
  for (size_t i = 0; i < n_; i++) {
    if (i == k) continue;
    X_.block(3 * i, 3 * k, 3, 3) = S.block(3 * j, 0, 3, 3);
    j++;
  }

  // Update the k-th row of Y, except Y(k, k)
  size_t i = 0;  // the i-th block of S
  for (size_t j = 0; j < n_; j++) {
    if (j == k) continue;
    X_.block(3 * k, 3 * j, 3, 3) = S.block(3 * i, 0, 3, 3).transpose();
    i++;
  }
}

}  // namespace solver
}  // namespace gopt
