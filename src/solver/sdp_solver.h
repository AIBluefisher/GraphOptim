#ifndef SOLVER_SDP_SOLVER_H_
#define SOLVER_SDP_SOLVER_H_

// #define EIGEN_USE_MKL_ALL

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
