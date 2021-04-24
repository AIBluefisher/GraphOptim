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

#ifndef SOLVER_RANK_RESTRICTED_SDP_SOLVER_H_
#define SOLVER_RANK_RESTRICTED_SDP_SOLVER_H_

#include <unordered_map>

#include <Eigen/Core>
#include <Eigen/SVD>

#include "solver/bcm_sdp_solver.h"
#include "solver/sdp_solver.h"

namespace gopt {
namespace solver {
// This algorithm has superior efficiency than the normal BCMSDPSolver in large
// scale SDP optimization problems. Instead of solving the original SDP problem:
// -----------------------------------------------------------
//                  min. tr(QX)
//                  s.t. X[i,i] = I_d, \forall i \in [n],
//                       X \succeq 0
// -----------------------------------------------------------
// This algorithm solves for the rank-restricted SDP problem:
// -----------------------------------------------------------
//                       min. tr(QY^TY)
//                       s.t. Y \in St(d,r)^n
// -----------------------------------------------------------
// where St(d,r)^n = {Y = [Y1 Y2 ... Yn] \in R^{rxdn}: Yi \in St(d,r)},
// and the Stiefel manifold St(d,r) = {Y \in R^{rxd}: Y^TY=I_d}
//
class RankRestrictedSDPSolver : public BCMSDPSolver {
 public:
  RankRestrictedSDPSolver(const size_t n, const size_t block_dim);

  RankRestrictedSDPSolver(const size_t n, const size_t block_dim,
                          const solver::SDPSolverOptions& options);

  void Solve(solver::Summary& summary) override;

  Eigen::MatrixXd GetSolution() const override;

  double EvaluateFuncVal() const override;
  double EvaluateFuncVal(const Eigen::MatrixXd& Y) const override;

  void SetOptimalY(const Eigen::MatrixXd& Y);

  Eigen::MatrixXd ComputeLambdaMatrix() const;
  const Eigen::MatrixXd& GetYStar() const; 
  const Eigen::MatrixXd ComputeQYt(const Eigen::MatrixXd& Y) const;

  void AugmentRank();

  size_t CurrentRank() const;

  Eigen::MatrixXd Project(const Eigen::MatrixXd& A) const;
  Eigen::MatrixXd Retract(
      const Eigen::MatrixXd& Y, const Eigen::MatrixXd& V) const;

  Eigen::MatrixXd EuclideanGradient(const Eigen::MatrixXd& Y) const;
  Eigen::MatrixXd RiemannianGradient(
      const Eigen::MatrixXd& Y, const Eigen::MatrixXd& nablaF_Y) const;
  Eigen::MatrixXd RiemannianGradient(const Eigen::MatrixXd& Y) const;

  Eigen::MatrixXd TangentSpaceProjection(
      const Eigen::MatrixXd& Y, const Eigen::MatrixXd& dotY) const;
  Eigen::MatrixXd SymBlockDiagProduct(
    const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
    const Eigen::MatrixXd &C) const;

  Eigen::MatrixXd Precondition(
      const Eigen::MatrixXd& Y, const Eigen::MatrixXd& dotY) const;
  
 private:
  // The rank-deficient first-order critical point
  // with row-block size is rank_*dim_.
  Eigen::MatrixXd Y_;

  // initial rank of Y_.
  size_t rank_;
};

}  // namespace solver
}  // namespace gopt

#endif  // SOLVER_RANK_RESTRICTED_SDP_SOLVER_H_
