#ifndef L1_ROTATION_GLOBAL_ESTIMATOR_H_
#define L1_ROTATION_GLOBAL_ESTIMATOR_H_

#include <unordered_map>

#include <Eigen/Core>
#include <Eigen/SparseCore>

#include "util/types.h"
#include "util/hash.h"

namespace gopt {

class L1RotationGlobalEstimator {
 public:
  struct L1RotationOptions {
    // Maximum number of times to run L1 minimization. L1 is very slow (compared
    // to L2), but is very robust to outliers. Typically only a few iterations
    // are needed in order for the solution to reside within the cone of
    // convergence for L2 solving.
    int max_num_l1_iterations = 5;

    // Average step size threshold to terminate the L1 minimization
    double l1_step_convergence_threshold = 0.001;
  };

  L1RotationGlobalEstimator(
      const int num_orientations, const int num_edges,
      const L1RotationOptions& options);

  void SetViewIdToIndex(
      const std::unordered_map<image_t, int>& view_id_to_index);
  void SetSparseMatrix(const Eigen::SparseMatrix<double>& sparse_matrix);

  bool SolveL1Regression(
      const std::unordered_map<ImagePair, TwoViewGeometry>& relative_rotations,
      std::unordered_map<image_t, Eigen::Vector3d>* global_rotations);

  // We keep one of the rotations as constant to remove the ambiguity of the
  // linear system.
  static const int kConstantRotationIndex = -1;

 private:
  // Update the global orientations using the current value in the
  // rotation_change.
  void UpdateGlobalRotations(
      std::unordered_map<image_t, Eigen::Vector3d>* global_rotations);

  // Computes the relative rotation error based on the current global
  // orientation estimates.
  void ComputeResiduals(
    const std::unordered_map<ImagePair, TwoViewGeometry>& relative_rotations,
    std::unordered_map<image_t, Eigen::Vector3d>* global_rotations);

  // Computes the average size of the most recent step of the algorithm.
  // The is the average over all non-fixed global_rotations_ of their
  // rotation magnitudes.
  double ComputeAverageStepSize();

  L1RotationOptions options_;

  // Map of image_ts to the corresponding positions of the view's orientation in
  // the linear system.
  std::unordered_map<image_t, int> view_id_to_index_;

  // The sparse matrix used to maintain the linear system. This is matrix A in
  // Ax = b.
  Eigen::SparseMatrix<double> sparse_matrix_;

  // x in the linear system Ax = b.
  Eigen::VectorXd tangent_space_step_;

  // b in the linear system Ax = b.
  Eigen::VectorXd tangent_space_residual_;
};

}  // namespace gopt

#endif  // L1_TANGENT_SPACE_ESTIMATOR_H_
