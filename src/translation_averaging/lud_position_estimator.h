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
