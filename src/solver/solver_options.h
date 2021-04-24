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
