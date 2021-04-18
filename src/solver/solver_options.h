#ifndef SOLVER_SOLVER_OPTIONS_H_
#define SOLVER_SOLVER_OPTIONS_H_

#include <iostream>

namespace gopt {
namespace solver {

enum SDPSolverType {
  RBR_BCM,
  RANK_DEFICIENT_BCM,
  RIEMANNIAN_STAIRCASE,
  SE_SYNC, // not implemented
  SHONAN   // not implemented
};

enum PreconditionerType {
  None,
  JACOBI,
  INCOMPLETE_CHOLESKY,
  REGULARIZED_CHOLESKY
};

struct RiemannianStaircaseOptions {
  size_t min_rank = 3;
  size_t max_rank = 10;

  size_t max_eigen_solver_iterations = 20;

  double min_eigenvalue_nonnegativity_tolerance = 1e-5;

  size_t num_Lanczos_vectors = 20;

  SDPSolverType local_solver_type = SDPSolverType::RANK_DEFICIENT_BCM;

  double gradient_tolerance = 1e-2;

  double preconditioned_gradient_tolerance = 1e-4;
};

struct SDPSolverOptions {
  // maximum iteration number
  size_t max_iterations = 500;

  // tolerance for convergence
  double tolerance = 1e-8;

  bool verbose = true;

  int num_threads = 8;

  SDPSolverType solver_type = RIEMANNIAN_STAIRCASE;

  PreconditionerType preconditioner_type = PreconditionerType::None;

  RiemannianStaircaseOptions riemannian_staircase_options;

  SDPSolverOptions(size_t max_iter = 500, double tol = 1e-8, bool log = true) {
    max_iterations = max_iter;
    tolerance = tol;
    verbose = log;
    solver_type = RIEMANNIAN_STAIRCASE;
  }

  SDPSolverOptions(const SDPSolverOptions& option) {
    max_iterations = option.max_iterations;
    tolerance = option.tolerance;
    verbose = option.verbose;
    solver_type = option.solver_type;
    riemannian_staircase_options = option.riemannian_staircase_options;
  }
};

}  // namespace solver
}  // namespace gopt

#endif  // SOLVER_SOLVER_OPTIONS_H_
