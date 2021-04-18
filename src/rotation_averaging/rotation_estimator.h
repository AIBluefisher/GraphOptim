#ifndef ROTATION_AVERAGING_ROTATION_ESTIMATOR_H_
#define ROTATION_AVERAGING_ROTATION_ESTIMATOR_H_

#include <algorithm>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <unordered_map>

#include "solver/solver_options.h"
#include "rotation_averaging/l1_rotation_global_estimator.h"
#include "rotation_averaging/irls_rotation_local_refiner.h"
#include "util/map_util.h"
#include "util/random.h"
#include "util/types.h"
#include "util/util.h"

namespace gopt {

// The recommended type of rotations solver is the Robust L1-L2 method. This
// method is scalable, extremely accurate, and very efficient. See the
// global_pose_estimation directory for more details.
enum class GlobalRotationEstimatorType : int {
  LAGRANGIAN_DUAL = 0,
  HYBRID = 1,
  ROBUST_L1L2 = 2
};

enum class GlobalRotationEstimatorInitMethod : int {
  RANDOM = 0,
  MAXIMUM_SPANNING_TREE = 1
};

struct RotationEstimatorOptions {
  GlobalRotationEstimatorType estimator_type =
      GlobalRotationEstimatorType::HYBRID;

  GlobalRotationEstimatorInitMethod init_method =
      GlobalRotationEstimatorInitMethod::RANDOM;

  solver::SDPSolverOptions sdp_solver_options;

  L1RotationGlobalEstimator::L1RotationOptions l1_options;

  IRLSRotationLocalRefiner::IRLSRefinerOptions irls_options;
};

// A generic class defining the interface for global rotation estimation
// methods. These methods take in as input the relative pairwise orientations
// and output estimates for the global orientation of each view.
class RotationEstimator {
 public:
  RotationEstimator() {}
  virtual ~RotationEstimator() {}
  // Input the view pairs containing relative rotations between matched
  // geometrically verified views and outputs a rotation estimate for each view.
  //
  // Returns true if the rotation estimation was a success, false if there was a
  // failure. If false is returned, the contents of rotations are undefined.
  virtual bool EstimateRotations(
      const std::unordered_map<ImagePair, TwoViewGeometry>& view_pairs,
      std::unordered_map<image_t, Eigen::Vector3d>* rotations) = 0;

 private:
  DISALLOW_COPY_AND_ASSIGN(RotationEstimator);
};

}  // namespace gopt

#endif  // ROTATION_AVERAGING_ROTATION_ESTIMATOR_H_
