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

#ifndef SOLVER_RIEMANNIAN_STAIRCASE_H_
#define SOLVER_RIEMANNIAN_STAIRCASE_H_

#include <memory>

#include <Eigen/Dense>
#include <Eigen/SparseCore>

#include "solver/rank_restricted_sdp_solver.h"

namespace gopt {
namespace solver {

// This struct is derived from SE-Sync.
// This is a lightweight struct used in conjunction with Spectra to compute
// the minimum eigenvalue and eigenvector of S - Lambda(X); it has a single
// nontrivial function, perform_op(x,y), that computes and returns the product
// y = (S - Lambda + sigma*I) x
template <typename Scalar_>
struct SMinusLambdaProdFunctor {
  using Scalar = Scalar_;
  
  std::shared_ptr<RankRestrictedSDPSolver> sdp_solver_;

  // Diagonal blocks of the matrix Lambda
  Eigen::MatrixXd Lambda_;

  // Number of rows and columns of the matrix B - Lambda
  size_t rows_;
  size_t cols_;

  // Dimensional parameter d of the special Euclidean group SE(d) over which
  // this synchronization problem is defined
  size_t dim_;
  double sigma_;

  // Constructor
  SMinusLambdaProdFunctor(
      std::shared_ptr<RankRestrictedSDPSolver> sdp_solver,
      double sigma = 0);

  size_t rows() const { return rows_; }
  size_t cols() const { return cols_; }

  // Matrix-vector multiplication operation
  void perform_op(const double* x, double* y) const;
};

template <typename Scalar_>
SMinusLambdaProdFunctor<Scalar_>::SMinusLambdaProdFunctor(
    std::shared_ptr<RankRestrictedSDPSolver> sdp_solver,
    double sigma)
    : sdp_solver_(sdp_solver), dim_(sdp_solver->Dimension()), sigma_(sigma) {
  rows_ = sdp_solver_->Dimension() * sdp_solver_->NumUnknowns();
  cols_ = sdp_solver_->Dimension() * sdp_solver_->NumUnknowns();

  // Compute and cache this on construction
  Lambda_ = sdp_solver_->ComputeLambdaMatrix();
}

template <typename Scalar_>
void SMinusLambdaProdFunctor<Scalar_>::perform_op(
    const double* x, double* y) const {
  Eigen::Map<Eigen::VectorXd> X(const_cast<double*>(x), cols_);
  Eigen::Map<Eigen::VectorXd> Y(y, rows_);

  Y = sdp_solver_->ComputeQYt(X.transpose());

#pragma omp parallel for
  for (size_t i = 0; i < sdp_solver_->NumUnknowns(); ++i) {
    Y.segment(i * dim_, dim_) -=
        Lambda_.block(0, i * dim_, dim_, dim_) *
        X.segment(i * dim_, dim_);
  }

  if (sigma_ != 0) {
    Y += sigma_ * X;
  }
}

class RiemannianStaircase : public SDPSolver{
 public:
  RiemannianStaircase(const size_t n, const size_t block_dim);
  RiemannianStaircase(const size_t n, const size_t block_dim,
                      const solver::SDPSolverOptions& options);

  void Solve(solver::Summary& summary) override;

  Eigen::MatrixXd GetSolution() const override;

  double EvaluateFuncVal() const override;
  double EvaluateFuncVal(const Eigen::MatrixXd& Y) const override;

  void SetCovariance(const Eigen::SparseMatrix<double>& Q) override;
  void SetAdjacentEdges(
      const std::unordered_map<size_t, std::vector<size_t>>& adj_edges) override;

 private:

  bool KKTVerification(
      double* min_eigenvalue, Eigen::VectorXd* min_eigenvector,
      size_t* num_iterations);
  
  bool EscapeSaddle(
      const double lambda_min, const Eigen::VectorXd& vector_min,
      double gradient_tolerance, double preconditioned_gradient_tolerance,
      Eigen::MatrixXd* Yplus);

  void RoundSolution();

  size_t n_;
  size_t dim_;

  SDPSolverOptions sdp_options_;

  std::shared_ptr<RankRestrictedSDPSolver> sdp_solver_;

  Eigen::MatrixXd R_;
};

}  // namespace solver
}  // namespace gopt

#endif  // SOLVER_RIEMANNIAN_STAIRCASE_H_
