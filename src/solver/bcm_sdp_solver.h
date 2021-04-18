#ifndef SOLVER_BCM_SDP_SOLVER_H_
#define SOLVER_BCM_SDP_SOLVER_H_

#include <iostream>
#include <iomanip>
#include <chrono>

#include <Eigen/Core>

#include "solver/sdp_solver.h"

namespace gopt {
namespace solver {

class BCMSDPSolver : public SDPSolver {
 public:
  BCMSDPSolver(const size_t n, const size_t block_dim) 
      : BCMSDPSolver(n, block_dim, solver::SDPSolverOptions()) {}

  BCMSDPSolver(const size_t n, const size_t block_dim,
               const solver::SDPSolverOptions& options)
      : SDPSolver(n, block_dim, options) {}

  // Kernel function to solve the SDP problem with BCM
  virtual void Solve(solver::Summary& summary) = 0;

  virtual Eigen::MatrixXd GetSolution() const = 0;

  virtual double EvaluateFuncVal() const = 0;
  virtual double EvaluateFuncVal(const Eigen::MatrixXd& Y) const = 0;

 protected:
  // Judge if algorithm converges
  virtual bool IsConverge(const double prev_funv, 
                          const double cur_funv,
                          const double tolerance,
                          double* error) const {
    *error = std::abs(prev_funv - cur_funv) / std::max(std::abs(prev_funv), 1.0);
    return *error <= tolerance;
  }

  // Log running information to terminal
  void LogToStd(
      const size_t& iter, const double& pre_funv, const double& cur_funv,
      const double& error, const double& time) {
    if (iter < 1) {
      std::cout << "\n" 
                << std::setw(11) << std::setfill(' ') << "Iter " 
                << std::setw(16) << std::setfill(' ') << "Prev " 
                << std::setw(16) << std::setfill(' ') << "Cur " 
                << std::setw(16) << std::setfill(' ') << "Error " 
                << std::setw(16) << std::setfill(' ') << "Time"
                << std::endl;
    } else {
      std::cout << std::setw(8) << std::setfill(' ') << iter 
                << std::setw(5) << std::setfill(' ') << " " 
                << std::setw(14) << std::setfill(' ') << pre_funv 
                << std::setw(5) << std::setfill(' ') << " " 
                << std::setw(12) << std::setfill(' ') << cur_funv 
                << std::setw(5) << std::setfill(' ') << " " 
                << std::setw(12) << std::setfill(' ') << error 
                << std::setw(2) << std::setfill(' ') << " " 
                << std::setw(10) << std::setfill(' ') << time
                << std::endl;
    }
  }
};

}  // namespace solver
} // namespace gopt

#endif  // SOLVER_BCM_SDP_SOLVER_H_
