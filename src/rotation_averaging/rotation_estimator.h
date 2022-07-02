// Copyright (C) 2014 The Regents of the University of California (Regents).
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials provided
//       with the distribution.
//
//     * Neither the name of The Regents or University of California nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Please contact the author of this library if you have any questions.
// Author: Chris Sweeney (cmsweeney@cs.ucsb.edu)

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
  bool verbose = true;

  GlobalRotationEstimatorType estimator_type =
      GlobalRotationEstimatorType::HYBRID;

  GlobalRotationEstimatorInitMethod init_method =
      GlobalRotationEstimatorInitMethod::RANDOM;

  solver::SDPSolverOptions sdp_solver_options;

  L1RotationGlobalEstimator::L1RotationOptions l1_options;

  IRLSRotationLocalRefiner::IRLSRefinerOptions irls_options;

  void Setup() {
    l1_options.verbose = verbose;
    irls_options.verbose = verbose;
    sdp_solver_options.verbose = verbose;
  }
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
