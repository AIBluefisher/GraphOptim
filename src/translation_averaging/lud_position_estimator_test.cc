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

#include "translation_averaging/lud_position_estimator.h"

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "geometry/align_point_clouds.h"
#include "geometry/rotation_utils.h"
#include "utils/types.h"
#include "utils/map_util.h"

namespace gopt {
namespace {

RandomNumberGenerator rng(63);

// Aligns positions to the ground truth positions via a similarity
// transformation.
void AlignPositions(
    const std::unordered_map<image_t, Eigen::Vector3d>& gt_positions,
    std::unordered_map<image_t, Eigen::Vector3d>* positions) {
  // Collect all positions into a vector.
  std::vector<Eigen::Vector3d> gt_pos, pos;
  for (const auto& gt_position : gt_positions) {
    gt_pos.push_back(gt_position.second);
    const Eigen::Vector3d& position = FindOrDie(*positions, gt_position.first);
    pos.push_back(position);
  }

  Eigen::Matrix3d rotation;
  Eigen::Vector3d translation;
  double scale;
  AlignPointCloudsUmeyama(pos, gt_pos, &rotation, &translation, &scale);

  // Apply the similarity transformation.
  for (auto& position : *positions) {
    position.second = scale * (rotation * position.second) + translation;
  }
}
} // namespace

class EstimatePositionsLeastUnsquaredDeviationTest : public ::testing::Test {
 public:
  void TestLUDPositionEstimator(
      const size_t num_views,
      const size_t num_view_pairs,
      const double pose_noise,
      const double position_tolerance) {
    // Set up the scene.
    SetupScene(num_views);
    GetTwoViewGeometries(num_view_pairs, pose_noise);

    // Estimate the positions.
    LUDPositionEstimator position_estimator(options_);
    std::unordered_map<image_t, Eigen::Vector3d> estimated_positions;
    EXPECT_TRUE(position_estimator.EstimatePositions(view_pairs_,
                                                     orientations_,
                                                     &estimated_positions));
    EXPECT_EQ(estimated_positions.size(), positions_.size());

    // Align the positions and measure the error.
    AlignPositions(positions_, &estimated_positions);

    std::vector<double> position_errors;
    for (const auto& position : positions_) {
      const Eigen::Vector3d& estimated_position =
          FindOrDie(estimated_positions, position.first);
      const double position_error =
          (position.second - estimated_position).norm();
      position_errors.push_back(position_error);
      EXPECT_LT(position_error, position_tolerance)
          << "\ng.t. position = " << position.second.transpose()
          << "\nestimated position = " << estimated_position.transpose();
    }

    double sum_errors = 0;
    for (const double error : position_errors) {
      sum_errors += error;
    }
    std::sort(position_errors.begin(), position_errors.end());

    const double mean_error = sum_errors / position_errors.size();
    LOG(INFO) << "Mean Position error: " << mean_error;
    LOG(INFO) << "Median Position error: " << position_errors[position_errors.size() / 2];
  }

 protected:
  void SetUp() {}

  void SetupScene(const int num_views) {
    // Create random views.
    for (int i = 0; i < num_views; i++) {
      // Create a random pose.
      orientations_[i] = 0.2 * rng.RandVector3d();
      positions_[i] = 10.0 * rng.RandVector3d();
    }
  }

  void GetTwoViewGeometries(const size_t num_view_pairs, const double pose_noise) {
    // Create a single connected component.
    std::vector<image_t> view_ids;
    view_ids.push_back(0);
    for (size_t i = 1; i < positions_.size(); i++) {
      const ImagePair view_id_pair(i - 1, i);
      view_pairs_[view_id_pair] = CreateTwoViewGeometry(view_id_pair, pose_noise);
      view_ids.push_back(i);
    }

    while (view_pairs_.size() < num_view_pairs) {
      std::random_shuffle(view_ids.begin(), view_ids.end());
      const ImagePair view_id_pair =
          (view_ids[0] < view_ids[1]) ? ImagePair(view_ids[0], view_ids[1])
                                      : ImagePair(view_ids[1], view_ids[0]);
      if (ContainsKey(view_pairs_, view_id_pair)) {
        continue;
      }

      view_pairs_[view_id_pair] = CreateTwoViewGeometry(view_id_pair, pose_noise);
    }
  }

  TwoViewGeometry CreateTwoViewGeometry(const ImagePair& view_id_pair,
                                const double pose_noise) {
    CHECK_LT(view_id_pair.first, view_id_pair.second);
    TwoViewGeometry two_view_geometry;

    // These objects will add noise to the relative pose.
    const Eigen::Vector2d noise = pose_noise * rng.RandVector2d();

    // Determine the relative rotation and add noise.
    two_view_geometry.rel_rotation =
      geometry::RelativeRotationFromTwoRotations(
        FindOrDie(orientations_, view_id_pair.first),
        FindOrDie(orientations_, view_id_pair.second),
        noise(0));

    // Determine the relative position and add noise.
    two_view_geometry.rel_translation =
      geometry::RelativeTranslationFromTwoPositions(
        FindOrDie(positions_, view_id_pair.first),
        FindOrDie(positions_, view_id_pair.second),
        FindOrDie(orientations_, view_id_pair.first),
        rng.RandVector3d().normalized(),
        geometry::DegToRad(noise(1)));

    return two_view_geometry;
  }

  LUDPositionEstimator::Options options_;
  std::unordered_map<image_t, Eigen::Vector3d> positions_;
  std::unordered_map<image_t, Eigen::Vector3d> orientations_;
  std::unordered_map<ImagePair, TwoViewGeometry> view_pairs_;
};

TEST_F(EstimatePositionsLeastUnsquaredDeviationTest, SmallTestNoNoise) {
  static const double kTolerance = 1e-2;
  static const size_t kNumViews = 4;
  static const size_t kNumViewPairs = 6;
  TestLUDPositionEstimator(kNumViews,
                           kNumViewPairs,
                           0.0,
                           kTolerance);
}

TEST_F(EstimatePositionsLeastUnsquaredDeviationTest, SmallTestWithNoise) {
  static const double kTolerance = 0.2;
  static const size_t kNumViews = 4;
  static const size_t kNumViewPairs = 6;
  static const double kPoseNoiseDegrees = 1.0;
  TestLUDPositionEstimator(kNumViews,
                           kNumViewPairs,
                           kPoseNoiseDegrees,
                           kTolerance);
}

}  // namespace gopt
