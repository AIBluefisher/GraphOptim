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

#include "solver/riemannian_staircase.h"

#include <algorithm>

#include "geometry/rotation_utils.h"
#include "Spectra/SymEigsSolver.h"

namespace gopt {
namespace solver {

RiemannianStaircase::RiemannianStaircase(const size_t n, const size_t block_dim)
  : RiemannianStaircase(n, block_dim, solver::SDPSolverOptions()) {}

RiemannianStaircase::RiemannianStaircase(
    const size_t n, const size_t block_dim,
    const solver::SDPSolverOptions& options)
  : SDPSolver(n, block_dim, options),
    n_(n),
    dim_(block_dim),
    sdp_options_(options),
    sdp_solver_(new RankRestrictedSDPSolver(n, block_dim, options)) {}

void RiemannianStaircase::Solve(solver::Summary& summary) {
  const RiemannianStaircaseOptions& riemannian_options =
      sdp_options_.riemannian_staircase_options;
  for (size_t i = riemannian_options.min_rank; i <= riemannian_options.max_rank; ++i) {
    LOG(INFO) << "Current rank: " << i;
    // Local search for the second critical point.
    summary = Summary();
    sdp_solver_->Solve(summary);

    // Verify global optimality.
    double min_eigenvalue = 0;
    Eigen::VectorXd min_eigenvector;
    size_t num_iterations;
    bool eigenvalue_converge =
        KKTVerification(&min_eigenvalue, &min_eigenvector, &num_iterations);

    if (!eigenvalue_converge) {
      LOG(INFO) << "Eigen value not converge";
      break;
    }

    if (min_eigenvalue > -riemannian_options.min_eigenvalue_nonnegativity_tolerance) {
      LOG(INFO) << "Found second order critical point!";
      
      break;
    }

    // Global optimality not satisfied, then augment the solution rank.
    sdp_solver_->AugmentRank();

    // Escape the saddle point from the obtained mininum eigenvector.
    LOG(INFO) << "Current solution is not second order critical point,"
              << " escaping saddle point!";
    Eigen::MatrixXd Yplus;
    if (EscapeSaddle(min_eigenvalue, min_eigenvector,
        riemannian_options.gradient_tolerance,
        riemannian_options.preconditioned_gradient_tolerance, &Yplus)) {
      sdp_solver_->SetOptimalY(Yplus);
    } else {
      LOG(WARNING) << "Escape saddle point failed!";
      break;
    }
  }

  // Rounding solution.
  LOG(INFO) << "Rounding Solutions";
  R_ = sdp_solver_->GetSolution();
  if (sdp_solver_->CurrentRank() > riemannian_options.min_rank) {
    RoundSolution();
  }
}

bool RiemannianStaircase::KKTVerification(
    double* min_eigenvalue,
    Eigen::VectorXd* min_eigenvector,
    size_t* num_iterations) {
  // First, compute the largest-magnitude eigenvalue of this matrix
  const Eigen::MatrixXd& Y = sdp_solver_->GetYStar();
  const auto& riemannian_options = sdp_options_.riemannian_staircase_options;
  
  SMinusLambdaProdFunctor<double> lm_op(sdp_solver_);

  Spectra::SymEigsSolver<SMinusLambdaProdFunctor<double>>
      largest_magnitude_eigensolver(
          lm_op, 1,
          std::min(riemannian_options.num_Lanczos_vectors, n_ * dim_));

  largest_magnitude_eigensolver.init();
  int num_converged = largest_magnitude_eigensolver.compute(
      Spectra::SortRule::LargestMagn,
      riemannian_options.max_eigen_solver_iterations, 1e-4,
      Spectra::SortRule::LargestMagn);

  // Check convergence and bail out if necessary
  if (num_converged != 1) {
    LOG(INFO) << "Not converge";
    return false;
  }

  double lambda_lm = largest_magnitude_eigensolver.eigenvalues()(0);
  LOG(INFO) << "Init eigenvalue: " << lambda_lm;

  if (lambda_lm < 0) {
    // The largest-magnitude eigenvalue is negative, and therefore also the
    // minimum eigenvalue, so just return this solution
    *min_eigenvalue = lambda_lm;
    *min_eigenvector = largest_magnitude_eigensolver.eigenvectors(1);
    (*min_eigenvector).normalize(); // Ensure that this is a unit vector
    return true;
  }

  // The largest-magnitude eigenvalue is positive, and is therefore the
  // maximum  eigenvalue.  Therefore, after shifting the spectrum of S - Lambda
  // by -2*lambda_lm (by forming S - Lambda - 2*lambda_max*I), the  shifted
  // spectrum will lie in the interval [lambda_min(A) - 2*  lambda_max(A),
  // -lambda_max*A]; in particular, the largest-magnitude eigenvalue of  S -
  // Lambda - 2*lambda_max*I is lambda_min - 2*lambda_max, with  corresponding
  // eigenvector v_min; furthermore, the condition number sigma of S - Lambda
  // -2*lambda_max is then upper-bounded by 2 :-).

  SMinusLambdaProdFunctor<double> min_shifted_op(sdp_solver_, -2 * lambda_lm);

  Spectra::SymEigsSolver<SMinusLambdaProdFunctor<double>>
      min_eigensolver(
          min_shifted_op, 1,
          std::min(riemannian_options.num_Lanczos_vectors, n_ * dim_));

  // If Y is a critical point of F, then Y^T is also in the null space of S -
  // Lambda(Y) (cf. Lemma 6 of the tech report), and therefore its rows are
  // eigenvectors corresponding to the eigenvalue 0.  In the case  that the
  // relaxation is exact, this is the *minimum* eigenvalue, and therefore the
  // rows of Y are exactly the eigenvectors that we're looking for.  On the
  // other hand, if the relaxation is *not* exact, then S - Lambda(Y) has at
  // least one strictly negative eigenvalue, and the rows of Y are *unstable
  // fixed points* for the Lanczos iterations.  Thus, we will take a slightly
  // "fuzzed" version of the first row of Y as an initialization for the Lanczos
  // iterations; this allows for rapid convergence in the case that the
  // relaxation is exact (since are starting close to a solution), while
  // simultaneously allowing the iterations to escape from this fixed point in
  // the case that the relaxation is not exact.
  Eigen::VectorXd v0 = Y.row(0).transpose();
  Eigen::VectorXd perturbation(v0.size());
  perturbation.setRandom();
  perturbation.normalize();
  Eigen::VectorXd xinit = v0 + (.03 * v0.norm()) * perturbation; // Perturb v0 by ~3%

  // Use this to initialize the eigensolver.
  min_eigensolver.init(xinit.data());

  // Now determine the relative precision required in the Lanczos method in
  // order to be able to estimate the smallest eigenvalue within an *absolute*
  // tolerance of 'min_eigenvalue_nonnegativity_tolerance'.
  num_converged = min_eigensolver.compute(
      Spectra::SortRule::LargestMagn,
      riemannian_options.max_eigen_solver_iterations,
      riemannian_options.min_eigenvalue_nonnegativity_tolerance / lambda_lm,
      Spectra::SortRule::LargestMagn);

  if (num_converged != 1) {
    return false;
  }

  *min_eigenvector = min_eigensolver.eigenvectors(1);
  (*min_eigenvector).normalize(); // Ensure that this is a unit vector
  *min_eigenvalue = min_eigensolver.eigenvalues()(0) + 2 * lambda_lm;
  *num_iterations = min_eigensolver.num_iterations();

  LOG(INFO) << "Found eigenvalue: " << (*min_eigenvalue);

  return true;
}

bool RiemannianStaircase::EscapeSaddle(
    const double lambda_min, const Eigen::VectorXd& vector_min,
    double gradient_tolerance, double preconditioned_gradient_tolerance,
    Eigen::MatrixXd* Yplus) {
  // v_min is an eigenvector corresponding to a negative eigenvalue of Q -
  // Lambda, so the KKT conditions for the semidefinite relaxation are not
  // satisfied; this implies that Y is a saddle point of the rank-restricted
  // semidefinite  optimization.  Fortunately, v_min can be used to compute a
  // descent  direction from this saddle point, as described in Theorem 3.9
  // of the paper "A Riemannian Low-Rank Method for Optimization over
  // Semidefinite  Matrices with Block-Diagonal Constraints". Define the vector
  // Ydot := e_{r+1} * v'; this is a tangent vector to the domain of the SDP
  // and provides a direction of negative curvature.
  const Eigen::MatrixXd& Y = sdp_solver_->GetYStar();
  // Function value at current iterate (saddle point).
  double FY = sdp_solver_->EvaluateFuncVal();

  // Relaxation rank at the NEXT level of the Riemannian Staircase, i.e. we
  // require that r = Y.rows() + 1.
  size_t r = sdp_solver_->CurrentRank();

  // Construct the corresponding representation of the saddle point Y in the
  // next level of the Riemannian Staircase by adding a row of 0's
  Eigen::MatrixXd Y_augmented = Y;

  Eigen::MatrixXd Ydot = Eigen::MatrixXd::Zero(r, Y.cols());
  Ydot.bottomRows<1>() = vector_min.transpose();

  // Set the initial step length to the greater of 10 times the distance needed
  // to arrive at a trial point whose gradient is large enough to avoid
  // triggering the gradient norm tolerance stopping condition (according to the
  // local second-order model), or at least 2^4 times the minimum admissible.
  // steplength,
  double alpha_min = 1e-6; // Minimum stepsize
  double alpha =
      std::max(16 * alpha_min, 10 * gradient_tolerance / fabs(lambda_min));

  // Vectors of trial stepsizes and corresponding function values
  std::vector<double> alphas;
  std::vector<double> fvals;

  /// Backtracking line search
  Eigen::MatrixXd Ytest;
  while (alpha >= alpha_min) {
    // Retract along the given tangent vector using the given stepsize
    Ytest = sdp_solver_->Retract(Y_augmented, alpha * Ydot);

    // Ensure that the trial point Ytest has a lower function value than
    // the current iterate Y, and that the gradient at Ytest is
    // sufficiently large that we will not automatically trigger the
    // gradient tolerance stopping criterion at the next iteration
    double FYtest = sdp_solver_->EvaluateFuncVal(Ytest);
    Eigen::MatrixXd grad_FYtest = sdp_solver_->RiemannianGradient(Ytest);
    double grad_FYtest_norm = grad_FYtest.norm();
    double preconditioned_grad_FYtest_norm =
        sdp_solver_->Precondition(Ytest, grad_FYtest).norm();

    // Record trial stepsize and function value
    alphas.push_back(alpha);
    fvals.push_back(FYtest);

    if ((FYtest < FY) && (grad_FYtest_norm > gradient_tolerance) &&
        (preconditioned_grad_FYtest_norm > preconditioned_gradient_tolerance)) {
      // Accept this trial point and return success
      *Yplus = Ytest;
      return true;
    }
    alpha /= 2;
  }

  // If control reaches here, we failed to find a trial point that satisfied
  // *both* the function decrease *and* gradient bounds.  In order to make
  // forward progress, we will fall back to accepting the trial point that
  // simply minimized the objective value, provided that it strictly *decreased*
  // the objective from the current (saddle) point

  // Find minimum function value from among the trial points
  auto fmin_iter = std::min_element(fvals.begin(), fvals.end());
  auto min_idx = std::distance(fvals.begin(), fmin_iter);

  double f_min = fvals[min_idx];
  double a_min = alphas[min_idx];

  if (f_min < FY) {
    // If this trial point strictly decreased the objective value, accept it and
    // return success
    *Yplus = sdp_solver_->Retract(Y_augmented, a_min * Ydot);
    return true;
  } else {
    // NO trial point decreased the objective value: we were unable to escape
    // the saddle point!
    return false;
  }
}

void RiemannianStaircase::RoundSolution() {
  // Finally, project each dxd rotation block to SO(d).
#pragma omp parallel for num_threads(sdp_options_.num_threads)
  for (size_t i = 0; i < n_; ++i) {
    R_.block(0, i * dim_, dim_, dim_) =
        geometry::ProjectToSOd(R_.block(0, i * dim_, dim_, dim_));
  }
}

Eigen::MatrixXd RiemannianStaircase::GetSolution() const {
  return R_;
}

double RiemannianStaircase::EvaluateFuncVal() const {
  return sdp_solver_->EvaluateFuncVal();
}

double RiemannianStaircase::EvaluateFuncVal(const Eigen::MatrixXd& Y) const {
  return sdp_solver_->EvaluateFuncVal(Y);
}

void RiemannianStaircase::SetCovariance(const Eigen::SparseMatrix<double>& Q) {
  sdp_solver_->SetCovariance(Q);
}

void RiemannianStaircase::SetAdjacentEdges(
    const std::unordered_map<size_t, std::vector<size_t>>& adj_edges) {
  sdp_solver_->SetAdjacentEdges(adj_edges);
}

}  // namespace solver
}  // namespace gopt
