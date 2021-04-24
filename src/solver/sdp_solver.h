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

#ifndef SOLVER_SDP_SOLVER_H_
#define SOLVER_SDP_SOLVER_H_

#include <iostream>
#include <unordered_map>

#include <glog/logging.h>
#include <Eigen/Core>
#include <Eigen/SparseCore>

#include "solver/solver_options.h"
#include "solver/summary.h"

namespace gopt {
namespace solver {

class SDPSolver {
 public:
  SDPSolver(const size_t n, const size_t block_dim)
      : SDPSolver(n, block_dim, solver::SDPSolverOptions()) { }

  SDPSolver(const size_t n, const size_t block_dim,
            const solver::SDPSolverOptions& options)
      : n_(n),
        dim_(block_dim),
        sdp_solver_options_(options) {
    Q_ = Eigen::SparseMatrix<double>(dim_ * n, dim_ * n);
  }

  virtual ~SDPSolver() {}

  void SetSolverOptions(const solver::SDPSolverOptions& options) {
    sdp_solver_options_ = options;
  }

  virtual void SetCovariance(const Eigen::SparseMatrix<double>& Q) {
    Q_ = Q;
  }

  virtual void SetAdjacentEdges(
      const std::unordered_map<size_t, std::vector<size_t>>& adj_edges) {
    adj_edges_ = adj_edges;
  }

  size_t NumUnknowns() const {
    return n_;
  }

  size_t Dimension() const {
    return dim_;
  }

  // Get the final optimal solution
  virtual Eigen::MatrixXd GetSolution() const = 0;

  virtual void Solve(solver::Summary& summary) = 0;

  virtual double EvaluateFuncVal() const = 0;
  virtual double EvaluateFuncVal(const Eigen::MatrixXd& Y) const = 0;

 protected:
  // number of unknown blocks.
  size_t n_;

  // the dimension of matrix sub-block.
  size_t dim_;

  // covariance matrix in SDP problem: min tr(QX).
  Eigen::SparseMatrix<double> Q_;

  std::unordered_map<size_t, std::vector<size_t>> adj_edges_;

  solver::SDPSolverOptions sdp_solver_options_;
};

}  // namespace solver
}  // namespace gopt

#endif  // SOLVER_SDP_SOLVER_H_
