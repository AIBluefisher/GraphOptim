#ifndef ROTATION_AVERARGING_LAGRANGE_DUAL_ROTATION_AVERAGING_H_
#define ROTATION_AVERARGING_LAGRANGE_DUAL_ROTATION_AVERAGING_H_

// #define EIGEN_USE_MKL_ALL

#include <iostream>
#include <memory>
#include <unordered_map>
#include <vector>
#include <chrono>

#include <glog/logging.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SparseCore>


#include "rotation_averaging/rotation_estimator.h"
#include "solver/sdp_solver.h"
#include "util/hash.h"
#include "util/types.h"

namespace gopt {

// Implementation of CVPR 2018 && PAMI 2019 paper:
// - Erikson. et.al. Rotation Averaging with strong duality.
// Given the pairwise relative rotations, retrive the globally optimal value
// of absolute orientations.
class LagrangeDualRotationEstimator : public RotationEstimator {
 public:
  LagrangeDualRotationEstimator(const int N, const int dim);
  LagrangeDualRotationEstimator(
      const int N, const int dim, const solver::SDPSolverOptions& option);

  void SetViewIdToIndex(const std::unordered_map<image_t, int>& view_id_to_index);

  void SetRAOption(const solver::SDPSolverOptions& option);

  const solver::Summary& GetRASummary() const;

  double GetErrorBound() const;

  // Estimate the absolute rotations, given pairs of relative rotations
  bool EstimateRotations(
      const std::unordered_map<ImagePair, TwoViewGeometry>& view_pairs,
      std::unordered_map<image_t, Eigen::Vector3d>* global_rotations) override;

  // Compute the upper bound of angular error alpha_max_
  // If for all |alpha_{ij}| < alpha_max_, the strong duality hold.
  void ComputeErrorBound(
      const std::unordered_map<ImagePair, TwoViewGeometry>& view_pairs);

 private:
  // Retrieve optimal solutions from matrix Y_
  void RetrieveRotations(
      const Eigen::MatrixXd& Y,
      std::unordered_map<image_t, Eigen::Vector3d>* global_rotations);

  void FillinRelativeGraph(
      const std::unordered_map<ImagePair, TwoViewGeometry>& view_pairs,
      Eigen::SparseMatrix<double>& R,
      std::unordered_map<size_t, std::vector<size_t>>& adj_edges);

  std::unique_ptr<solver::SDPSolver> CreateSDPSolver(const int n, const int dim);

 private:
  solver::SDPSolverOptions options_;

  solver::Summary summary_;

  // number of images/frames
  int images_num_;

  int dim_;

  // the compact matrix representation in Equ.(9) of Eriksson's paper
  Eigen::SparseMatrix<double> R_;

  // the optimized variable of (DD) problem
  Eigen::MatrixXd Y_;

  // upper bound for strong duality hold
  double alpha_max_;

  // this hash table is used for non-continuous index, such as
  // unordered internet datasets that composed of many unconnected components
  std::unordered_map<image_t, int> view_id_to_index_;
};

}  // namespace gopt

#endif  // ROTATION_AVERARGING_LAGRANGE_DUAL_ROTATION_AVERAGING_H_