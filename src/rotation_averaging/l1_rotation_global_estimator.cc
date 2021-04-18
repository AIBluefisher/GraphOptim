#include "rotation_averaging/l1_rotation_global_estimator.h"

#include <glog/logging.h>
#include <Eigen/SparseCore>

#include "geometry/rotation_utils.h"
#include "rotation_averaging/internal/rotation_estimator_util.h"
#include "solver/l1_solver.h"
#include "util/map_util.h"
#include "util/timer.h"

namespace gopt {

L1RotationGlobalEstimator::L1RotationGlobalEstimator(
    const int num_orientations, const int num_edges,
    const L1RotationOptions& options)
    : options_(options) {
  tangent_space_step_.resize((num_orientations - 1) * 3);
  tangent_space_residual_.resize(num_edges * 3);
}

void L1RotationGlobalEstimator::SetViewIdToIndex(
    const std::unordered_map<image_t, int>& view_id_to_index) {
  view_id_to_index_ = view_id_to_index;
}

void L1RotationGlobalEstimator::SetSparseMatrix(
    const Eigen::SparseMatrix<double>& sparse_matrix) {
  sparse_matrix_ = sparse_matrix;
}

bool L1RotationGlobalEstimator::SolveL1Regression(
    const std::unordered_map<ImagePair, TwoViewGeometry>& relative_rotations,
    std::unordered_map<image_t, Eigen::Vector3d>* global_rotations) {
  const int num_edges = relative_rotations.size();

  CHECK_NOTNULL(global_rotations);
  CHECK_GT(global_rotations->size(), 0);
  CHECK_GT(num_edges, 0);

  if (view_id_to_index_.empty()) {
    internal::ViewIdToAscentIndex(*global_rotations, &view_id_to_index_);
  }

  if (sparse_matrix_.rows() == 0) {
    internal::SetupLinearSystem(
        relative_rotations, (*global_rotations).size(),
        view_id_to_index_, &sparse_matrix_);
  }

  L1Solver<Eigen::SparseMatrix<double>>::Options l1_solver_options;
  l1_solver_options.max_num_iterations = 5;
  L1Solver<Eigen::SparseMatrix<double> > l1_solver(
      l1_solver_options, sparse_matrix_);

  tangent_space_step_.setZero();
  ComputeResiduals(relative_rotations, global_rotations);

  Timer timer;
  timer.Start();
  for (int i = 0; i < options_.max_num_l1_iterations; i++) {
    l1_solver.Solve(tangent_space_residual_, &tangent_space_step_);
    UpdateGlobalRotations(global_rotations);
    ComputeResiduals(relative_rotations, global_rotations);

    double avg_step_size = ComputeAverageStepSize();

    if (avg_step_size <= options_.l1_step_convergence_threshold) {
      break;
    }
    l1_solver_options.max_num_iterations *= 2;
    l1_solver.SetMaxIterations(l1_solver_options.max_num_iterations);
  }
  timer.Pause();

  LOG(INFO) << "Total time [L1Regression]: "
            << timer.ElapsedMicroSeconds() * 1e-3 << " ms.";

  return true;
}

void L1RotationGlobalEstimator::UpdateGlobalRotations(
    std::unordered_map<image_t, Eigen::Vector3d>* global_rotations) {
  for (auto& rotation : *global_rotations) {
    const int view_index = FindOrDie(view_id_to_index_, rotation.first) - 1;
    if (view_index == kConstantRotationIndex) {
      continue;
    }

    // Apply the rotation change to the global orientation.
    const Eigen::Vector3d& rotation_change =
        tangent_space_step_.segment<3>(3 * view_index);
    rotation.second = geometry::MultiplyRotations(rotation.second, rotation_change);
  }
}

void L1RotationGlobalEstimator::ComputeResiduals(
    const std::unordered_map<ImagePair, TwoViewGeometry>& relative_rotations,
    std::unordered_map<image_t, Eigen::Vector3d>* global_rotations) {
  int rotation_error_index = 0;

  for (const auto& relative_rotation : relative_rotations) {
    const Eigen::Vector3d& relative_rotation_aa = relative_rotation.second.rotation_2;
    const Eigen::Vector3d& rotation1 =
        FindOrDie(*global_rotations, relative_rotation.first.first);
    const Eigen::Vector3d& rotation2 =
        FindOrDie(*global_rotations, relative_rotation.first.second);

    // Compute the relative rotation error as:
    //   R_err = R2^t * R_12 * R1.
    tangent_space_residual_.segment<3>(3 * rotation_error_index) =
        geometry::MultiplyRotations(-rotation2,
            geometry::MultiplyRotations(relative_rotation_aa, rotation1));
    ++rotation_error_index;
  }
}

double L1RotationGlobalEstimator::ComputeAverageStepSize() {
  // compute the average step size of the update in tangent_space_step_
  const int num_vertices = tangent_space_step_.size() / 3;
  double delta_V = 0;
  for (int k = 0; k < num_vertices; ++k) {
    delta_V += tangent_space_step_.segment<3>(3 * k).norm();
  }
  return delta_V / num_vertices;
}

}  // namespace gopt
