#ifndef TRANSLATION_AVERAGING_POSITION_ESTIMATOR_H_
#define TRANSLATION_AVERAGING_POSITION_ESTIMATOR_H_

#include <unordered_map>

#include <Eigen/Core>

#include "util/types.h"
#include "util/util.h"
#include "util/hash.h"

namespace gopt {

enum class PositionEstimatorType : int {
  LUD = 0,
  BATA = 1
};

struct PositionEstimatorOptions {
  PositionEstimatorType estimator_type = PositionEstimatorType::LUD;

  bool verbose = true;

  // Options for ADMM QP solver.
  int max_num_iterations = 400;

  // Maximum number of reweighted iterations.
  int max_num_reweighted_iterations = 10;

  // A measurement for convergence criterion.
  double convergence_criterion = 1e-4;
};

// A generic class defining the interface for global position estimation
// methods. These methods take in as input the (global/absolute) orientation
// estimated for each camera and pairwise translation directions between pairs
// of cameras. Additional information such as track/correspondences can also be
// passed in as needed, but those will be specific to the subclass
// implementation.
class PositionEstimator {
 public:
  PositionEstimator() {}
  virtual ~PositionEstimator() {}

  // Input the view pairs containing relative poses between matched
  // geometrically verified views, as well as the global (absolute) orientations
  // of the camera that were previously estimated.
  //
  // Returns true if the position estimation was a success, false if there was a
  // failure. If false is returned, the contents of positions are undefined.
  virtual bool EstimatePositions(
      const std::unordered_map<ImagePair, TwoViewGeometry>& view_pairs,
      const std::unordered_map<image_t, Eigen::Vector3d>& orientation,
      std::unordered_map<image_t, Eigen::Vector3d>* positions) = 0;

 private:
  DISALLOW_COPY_AND_ASSIGN(PositionEstimator);
};

}  // namespace gopt

#endif  // TRANSLATION_AVERAGING_POSITION_ESTIMATOR_H_
