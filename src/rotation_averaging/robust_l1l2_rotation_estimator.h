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
    bool verbose = true;
    L1RotationGlobalEstimator::L1RotationOptions l1_options;
    IRLSRotationLocalRefiner::IRLSRefinerOptions irls_options;

    void Setup() {
      l1_options.verbose = verbose;
      irls_options.verbose = verbose;
    }
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
