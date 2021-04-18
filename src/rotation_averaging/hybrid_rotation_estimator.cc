#include "rotation_averaging/hybrid_rotation_estimator.h"

#include <ceres/rotation.h>
#include <glog/logging.h>
#include <omp.h>

#include <Eigen/Eigenvalues>
#include <Eigen/QR>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>

#include "Spectra/MatOp/SparseSymMatProd.h"
#include "Spectra/SymEigsSolver.h"

#include "rotation_averaging/internal/rotation_estimator_util.h"
#include "solver/bcm_sdp_solver.h"
#include "solver/rbr_sdp_solver.h"
#include "solver/rank_restricted_sdp_solver.h"
#include "solver/riemannian_staircase.h"

namespace gopt {

HybridRotationEstimator::HybridRotationEstimator(
    const int N, const int dim)
    : HybridRotationEstimator(N, dim, HybridRotationEstimatorOptions()) {}

HybridRotationEstimator::HybridRotationEstimator(
    const int N, const int dim,
    const HybridRotationEstimator::HybridRotationEstimatorOptions& options)
    : options_(options),
      images_num_(N),
      dim_(dim),
      ld_rotation_estimator_(new LagrangeDualRotationEstimator(
          N, dim, options.sdp_solver_options)),
      irls_rotation_refiner_(nullptr) {}

bool HybridRotationEstimator::EstimateRotations(
    const std::unordered_map<ImagePair, TwoViewGeometry>& view_pairs,
    std::unordered_map<image_t, Eigen::Vector3d>* global_rotations) {
  const int N = images_num_;

  CHECK_NOTNULL(global_rotations);
  CHECK_GT(N, 0);
  CHECK_EQ(N, (*global_rotations).size());
  CHECK_GT(view_pairs.size(), 0);

  irls_rotation_refiner_.reset(
      new IRLSRotationLocalRefiner(N, view_pairs.size(), options_.irls_options));
  
  internal::ViewIdToAscentIndex(*global_rotations, &view_id_to_index_);
  ld_rotation_estimator_->SetViewIdToIndex(view_id_to_index_);

  Eigen::SparseMatrix<double> sparse_matrix;
  internal::SetupLinearSystem(
      view_pairs, (*global_rotations).size(),
      view_id_to_index_, &sparse_matrix);
  irls_rotation_refiner_->SetViewIdToIndex(view_id_to_index_);
  irls_rotation_refiner_->SetSparseMatrix(sparse_matrix);

  // Estimate global rotations that resides within the cone of 
  // convergence for IRLS.
  LOG(INFO) << "Estimating Rotations Using LagrangeDual";
  ld_rotation_estimator_->EstimateRotations(view_pairs, global_rotations);

  // Refine the globally optimal result by IRLS.
  Eigen::VectorXd tangent_space_step;
  GlobalRotationsToTangentSpace(*global_rotations, &tangent_space_step);
  irls_rotation_refiner_->SetInitTangentSpaceStep(tangent_space_step);

  LOG(INFO) << "Refining Global Rotations";
  irls_rotation_refiner_->SolveIRLS(view_pairs, global_rotations);

  return true;
}

void HybridRotationEstimator::GlobalRotationsToTangentSpace(
    const std::unordered_map<image_t, Eigen::Vector3d>& global_rotations,
    Eigen::VectorXd* tangent_space_step) {
  (*tangent_space_step).resize((global_rotations.size() - 1) * 3);

  for (const auto& rotation : global_rotations) {
    const int view_index = FindOrDie(view_id_to_index_, rotation.first) - 1;

    if (view_index == IRLSRotationLocalRefiner::kConstantRotationIndex) {
      continue;
    }

    (*tangent_space_step).segment<3>(3 * view_index) = rotation.second;
  }
}

}  // namespace gopt
