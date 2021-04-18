#ifndef TRANSLATION_AVERAGING_LUD_POSITION_ESTIMATOR_H_
#define TRANSLATION_AVERAGING_LUD_POSITION_ESTIMATOR_H_

#include "translation_averaging/position_estimator.h"

namespace gopt {
// Least Unsquared Deviation (LUD) position estimator.
class LUDPositionEstimator : public PositionEstimator {
 public:
  struct Options {
    // Options for ADMM QP solver.
    int max_num_iterations = 400;

    // Maximum number of reweighted iterations.
    int max_num_reweighted_iterations = 10;

    // A measurement for convergence criterion.
    double convergence_criterion = 1e-4;
  };

  LUDPositionEstimator(const LUDPositionEstimator::Options& options);

  // Returns true if the optimization was a success, false if there was a
  // failure.
  bool EstimatePositions(
      const std::unordered_map<ImagePair, TwoViewGeometry>& view_pairs,
      const std::unordered_map<image_t, Eigen::Vector3d>& orientation,
      std::unordered_map<image_t, Eigen::Vector3d>* positions);

 private:
  void InitializeIndexMapping(
      const std::unordered_map<ImagePair, TwoViewGeometry>& view_pairs,
      const std::unordered_map<image_t, Eigen::Vector3d>& orientations);

  // Creates camera to camera constraints from relative translations.
  void SetupConstraintMatrix(
      const std::unordered_map<ImagePair, TwoViewGeometry>& view_pairs,
      const std::unordered_map<image_t, Eigen::Vector3d>& orientations);

  const LUDPositionEstimator::Options options_;

  std::unordered_map<ImagePair, int> view_id_pair_to_index_;
  std::unordered_map<image_t, int> view_id_to_index_;
  static const int kConstantViewIndex = -3;

  Eigen::SparseMatrix<double> constraint_matrix_;

  friend class EstimatePositionsLeastUnsquaredDeviationTest;

  DISALLOW_COPY_AND_ASSIGN(LUDPositionEstimator);
};

}  // namespace gopt

#endif  // TRANSLATION_AVERAGING_LUD_POSITION_ESTIMATOR_H_
