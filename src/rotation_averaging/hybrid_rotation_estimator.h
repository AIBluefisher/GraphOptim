#ifndef ROTATION_AVERAGING_HYBRID_ROTATION_AVERAGING_H_
#define ROTATION_AVERAGING_HYBRID_ROTATION_AVERAGING_H_

#include <memory>
#include <unordered_map>

#include "rotation_averaging/rotation_estimator.h"
#include "rotation_averaging/irls_rotation_local_refiner.h"
#include "rotation_averaging/lagrange_dual_rotation_estimator.h"
#include "solver/sdp_solver.h"
#include "solver/solver_options.h"
#include "util/hash.h"
#include "util/types.h"

namespace gopt {

class HybridRotationEstimator : public RotationEstimator {
 public:
  struct HybridRotationEstimatorOptions {
    solver::SDPSolverOptions sdp_solver_options;
    IRLSRotationLocalRefiner::IRLSRefinerOptions irls_options;
  };

  HybridRotationEstimator(const int N, const int dim);
  HybridRotationEstimator(
      const int N, const int dim,
      const HybridRotationEstimator::HybridRotationEstimatorOptions& options);

  // Estimate the absolute rotations, given pairs of relative rotations
  bool EstimateRotations(
      const std::unordered_map<ImagePair, TwoViewGeometry>& view_pairs,
      std::unordered_map<image_t, Eigen::Vector3d>* global_rotations) override;

 private:
  void GlobalRotationsToTangentSpace(
    const std::unordered_map<image_t, Eigen::Vector3d>& global_rotations,
    Eigen::VectorXd* tangent_space_step);

  HybridRotationEstimatorOptions options_;

  // number of images/frames
  int images_num_;

  int dim_;

  // this hash table is used for non-continuous index, such as
  // unordered internet datasets that composed of many unconnected components
  std::unordered_map<image_t, int> view_id_to_index_;

  std::unique_ptr<LagrangeDualRotationEstimator> ld_rotation_estimator_;
  std::unique_ptr<IRLSRotationLocalRefiner> irls_rotation_refiner_;
};

}  // namespace gopt

#endif  // ROTATION_AVERAGING_HYBRID_ROTATION_AVERAGING_H_
