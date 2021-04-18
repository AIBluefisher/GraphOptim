#ifndef ROTATION_AVERAGING_ROBUST_L1L2_ROTATION_ESTIMATOR_H_
#define ROTATION_AVERAGING_ROBUST_L1L2_ROTATION_ESTIMATOR_H_

#include <memory>
#include <unordered_map>

#include "rotation_averaging/rotation_estimator.h"
#include "rotation_averaging/irls_rotation_local_refiner.h"
#include "rotation_averaging/l1_rotation_global_estimator.h"
#include "solver/sdp_solver.h"
#include "solver/solver_options.h"
#include "util/hash.h"
#include "util/types.h"

namespace gopt {

class RobustL1L2RotationEstimator : public RotationEstimator {
 public:
  struct RobustL1L2RotationEstimatorOptions {
    L1RotationGlobalEstimator::L1RotationOptions l1_options;
    IRLSRotationLocalRefiner::IRLSRefinerOptions irls_options;
  };

  RobustL1L2RotationEstimator(
      const RobustL1L2RotationEstimatorOptions& options);

  // Estimate the absolute rotations, given pairs of relative rotations
  bool EstimateRotations(
      const std::unordered_map<ImagePair, TwoViewGeometry>& view_pairs,
      std::unordered_map<image_t, Eigen::Vector3d>* global_rotations) override;

 private:
  void GlobalRotationsToTangentSpace(
    const std::unordered_map<image_t, Eigen::Vector3d>& global_rotations,
    Eigen::VectorXd* tangent_space_step);

  RobustL1L2RotationEstimatorOptions options_;

  // this hash table is used for non-continuous index, such as
  // unordered internet datasets that composed of many unconnected components
  std::unordered_map<image_t, int> view_id_to_index_;

  std::unique_ptr<L1RotationGlobalEstimator> l1_rotation_estimator_;
  std::unique_ptr<IRLSRotationLocalRefiner> irls_rotation_refiner_;
};

}  // namespace gopt

#endif  // ROTATION_AVERAGING_ROBUST_L1L2_ROTATION_ESTIMATOR_H_
