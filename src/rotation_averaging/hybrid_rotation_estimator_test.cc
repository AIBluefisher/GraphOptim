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

#include "rotation_averaging/hybrid_rotation_estimator.h"

#include <ceres/rotation.h>

#include <cstring>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <fstream>
#include <string>
#include <unordered_map>

#include <glog/logging.h>
#include <gtest/gtest.h>

#include "geometry/rotation_utils.h"
#include "math/distribution.h"
#include "util/map_util.h"
#include "util/random.h"
#include "util/timer.h"

using namespace Eigen;

namespace gopt {

class HybridRotationAveragingTest : public ::testing::Test {
 protected:
  std::unordered_map<image_t, Vector3d> orientations_;
  std::unordered_map<ImagePair, TwoViewGeometry> view_pairs_;

 public:
  void TestHybridRotationEstimator(
      const int num_views, const int num_view_pairs,
      const double relative_rotation_noise, const double mean,
      const double variance, const double rotation_tolerance_degrees) {
    // At least a cycle graph
    ASSERT_LE(num_views + 1, num_view_pairs);

    // Set up the cameras
    CreateGTOrientations(num_views);
    CreateRelativeRotations(num_view_pairs, relative_rotation_noise, mean,
                            variance);

    // Initialize estimated orientations
    std::unordered_map<image_t, Eigen::Vector3d> estimated_orientations;
    // Or initialized randomly.
    this->InitializeRotationsFromSpanningTree(estimated_orientations);

    // Estimate global orientations.
    HybridRotationEstimator::HybridRotationEstimatorOptions options;

    solver::SDPSolverOptions sdp_options;
    sdp_options.max_iterations = 100;
    sdp_options.tolerance = 1e-8;
    sdp_options.verbose = true;

    IRLSRotationLocalRefiner::IRLSRefinerOptions irls_options;
    irls_options.max_num_irls_iterations = 10;

    options.sdp_solver_options = sdp_options;
    options.irls_options = irls_options;

    HybridRotationEstimator rotation_estimator(num_views, 3, options);

    Timer timer;
    timer.Start();
    EXPECT_TRUE(rotation_estimator.EstimateRotations(view_pairs_,
                                                     &estimated_orientations));
    EXPECT_EQ(estimated_orientations.size(), orientations_.size());
    timer.Pause();
    LOG(INFO) << "Elapsed time: " << timer.ElapsedSeconds();

    // LOG(INFO) << "Align the rotations and measure the error";
    // // Align the rotations and measure the error
    // geometry::AlignOrientations(orientations_, &estimated_orientations);

    double sum_angular_error = 0.0;
    double min_angular_error = std::numeric_limits<double>::max();
    double max_angular_error = 0.0;

    for (unsigned i = 0; i < orientations_.size(); i++) {
      const auto& rotation = FindOrDie(orientations_, i);
      const Vector3d& estimated_rotation = FindOrDie(estimated_orientations, i);
      const Vector3d relative_rotation =
          geometry::RelativeRotationFromTwoRotations(estimated_rotation,
                                                     rotation, 0.0);
      const double angular_error = geometry::RadToDeg(relative_rotation.norm());

      sum_angular_error += angular_error;
      min_angular_error = std::min(min_angular_error, angular_error);
      max_angular_error = std::max(max_angular_error, angular_error);
      // EXPECT_LT(angular_error, rotation_tolerance_degrees)
      // std::cout << "\n";
      // LOG(INFO) << i << "-th GT        rotation angle = "
      //           << RadToDeg(rotation.norm());
      // LOG(INFO) << i << "-th Estimated rotation angle = "
      //           << RadToDeg(estimated_rotation.norm());
      // LOG(INFO) << i << "-th Angular Residual         = " << angular_error;
    }

    std::cout << "\n";
    LOG(INFO) << "Average Angular Residual: "
              << sum_angular_error / orientations_.size();
    LOG(INFO) << "Maximum Angular Residual: " << max_angular_error;
    LOG(INFO) << "Minimum Angular Residual: " << min_angular_error;
  }

 protected:
  void SetUp() override {}

  void CreateGTOrientations(const int num_views) {
    double rotated_angle_rad = 0.0;
    Eigen::Vector3d unit_axis(0.0, 0.0, 1.0);

    for (int i = 0; i < num_views; i++) {
      // Rotation about the z-axis by 2π/n rad and
      // by construction, forming a cycle graph
      Eigen::Vector3d angle_axis = rotated_angle_rad * unit_axis;
      orientations_[i] = angle_axis;
      rotated_angle_rad += (2 * M_PI / num_views);
    }
  }

  void CreateRelativeRotations(const int num_view_pairs, const double noise,
                               const double mean, const double variance) {
    RandomNumberGenerator rng;
    NormalDistribution normal_distribution(mean, variance);
    const double relative_rotation_noise = normal_distribution.Eval(noise);
    const double noise_factor = 2.0;

    // Create a set of view id pairs that will contain a spanning tree
    for (size_t i = 1; i < orientations_.size(); i++) {
      const double x = rng.RandDouble(std::numeric_limits<double>::min(),
                                      std::numeric_limits<double>::max());
      const double real_noise = normal_distribution.Eval(x);

      const ImagePair view_id_pair(i - 1, i);
      view_pairs_[view_id_pair].visibility_score = geometry::rng.RandInt(1, 10);
      view_pairs_[view_id_pair].rotation_2 =
          geometry::RelativeRotationFromTwoRotations(
              FindOrDie(orientations_, view_id_pair.first),
              FindOrDie(orientations_, view_id_pair.second),
              noise_factor * real_noise /
                  view_pairs_[view_id_pair].visibility_score);
    }

    // Add random edges
    while (view_pairs_.size() < (unsigned)num_view_pairs) {
      ImagePair view_id_pair(
          geometry::rng.RandInt(0, orientations_.size() - 1),
          geometry::rng.RandInt(0, orientations_.size() - 1));

      // Ensure the first id is smaller than second id &&
      // do not add the views that already exists
      if (view_id_pair.first >= view_id_pair.second ||
          ContainsKey(view_pairs_, view_id_pair)) {
        continue;
      }

      view_pairs_[view_id_pair].visibility_score = geometry::rng.RandInt(1, 10);
      view_pairs_[view_id_pair].rotation_2 =
          geometry::RelativeRotationFromTwoRotations(
              FindOrDie(orientations_, view_id_pair.first),
              FindOrDie(orientations_, view_id_pair.second),
              noise_factor * relative_rotation_noise /
                  view_pairs_[view_id_pair].visibility_score);
    }
  }

  void InitializeRotationsFromSpanningTree(
      std::unordered_map<image_t, Vector3d>& initial_orientations) {
    // initial_orientations[0] = Vector3d::Zero();
    // for (size_t i = 1; i < orientations_.size(); i++) {
    //   initial_orientations[i] = geometry::ApplyRelativeRotation(
    //       FindOrDie(initial_orientations, i - 1),
    //       FindOrDieNoPrint(view_pairs_, ImagePair(i - 1, i)).rotation_2);
    // }
    for (size_t i = 0; i < orientations_.size(); i++) {
      initial_orientations[i] = Eigen::Vector3d::Zero();
    }
  }
};

TEST_F(HybridRotationAveragingTest, smallTestNoNoise) {
  const int num_views = 4;
  const int num_view_pairs = 6;
  const double noise = 1.0;
  const double mean = 0.0;
  const double variance = 0.1;
  const double rotation_tolerance_degrees = 4.0;
  TestHybridRotationEstimator(num_views, num_view_pairs, noise, mean,
                                    variance, rotation_tolerance_degrees);
}

TEST_F(HybridRotationAveragingTest, TwentyViewsTestWithSmallNoise) {
  const int num_views = 20;
  const int num_view_pairs = 30;
  const double noise = 1.0;
  const double mean = 0.0;
  const double variance = 0.2;
  const double rotation_tolerance_degrees = 4.0;
  TestHybridRotationEstimator(num_views, num_view_pairs, noise, mean,
                                    variance, rotation_tolerance_degrees);
}

TEST_F(HybridRotationAveragingTest, TwentyViewsTestWithLargeNoise) {
  const int num_views = 20;
  const int num_view_pairs = 30;
  const double noise = 1.0;
  const double mean = 0.0;
  const double variance = 0.5;
  const double rotation_tolerance_degrees = 4.0;
  TestHybridRotationEstimator(num_views, num_view_pairs, noise, mean,
                                    variance, rotation_tolerance_degrees);
}

TEST_F(HybridRotationAveragingTest, FiftyViewsTestWithSmallNoise) {
  const int num_views = 50;
  const int num_view_pairs = 75;
  const double noise = 1.0;
  const double mean = 0.0;
  const double variance = 0.2;
  const double rotation_tolerance_degrees = 4.0;
  TestHybridRotationEstimator(num_views, num_view_pairs, noise, mean,
                                    variance, rotation_tolerance_degrees);
}

TEST_F(HybridRotationAveragingTest, FiftyViewsTestWithLargeNoise) {
  const int num_views = 50;
  const int num_view_pairs = 75;
  const double noise = 1.0;
  const double mean = 0.0;
  const double variance = 0.5;
  const double rotation_tolerance_degrees = 4.0;
  TestHybridRotationEstimator(num_views, num_view_pairs, noise, mean,
                                    variance, rotation_tolerance_degrees);
}

TEST_F(HybridRotationAveragingTest, OneHundredViewsTestWithSmallNoise) {
  const int num_views = 100;
  const int num_view_pairs = 300;
  const double noise = 1.0;
  const double mean = 0.0;
  const double variance = 0.2;
  const double rotation_tolerance_degrees = 1e-8;
  TestHybridRotationEstimator(num_views, num_view_pairs, noise, mean,
                                    variance, rotation_tolerance_degrees);
}

TEST_F(HybridRotationAveragingTest, OneHundredViewsTestWithLargeNoise) {
  const int num_views = 100;
  const int num_view_pairs = 300;
  const double noise = 1.0;
  const double mean = 0.0;
  const double variance = 0.5;
  const double rotation_tolerance_degrees = 4.0;
  TestHybridRotationEstimator(num_views, num_view_pairs, noise, mean,
                                    variance, rotation_tolerance_degrees);
}

TEST_F(HybridRotationAveragingTest, TwoHundredViewsTestWithSmallNoise) {
  const int num_views = 200;
  const int num_view_pairs = 400;
  const double noise = 1.0;
  const double mean = 0.0;
  const double variance = 0.2;
  const double rotation_tolerance_degrees = 4.0;
  TestHybridRotationEstimator(num_views, num_view_pairs, noise, mean,
                                    variance, rotation_tolerance_degrees);
}

TEST_F(HybridRotationAveragingTest, TwoHundredViewsTestWithLargeNoise) {
  const int num_views = 200;
  const int num_view_pairs = 400;
  const double noise = 1.0;
  const double mean = 0.0;
  const double variance = 0.5;
  const double rotation_tolerance_degrees = 4.0;
  TestHybridRotationEstimator(num_views, num_view_pairs, noise, mean,
                                    variance, rotation_tolerance_degrees);
}

TEST_F(HybridRotationAveragingTest, FiveHundredViewsTestWithSmallNoise) {
  const int num_views = 500;
  const int num_view_pairs = 2000;
  const double noise = 1.0;
  const double mean = 0.0;
  const double variance = 0.2;
  const double rotation_tolerance_degrees = 4.0;
  TestHybridRotationEstimator(num_views, num_view_pairs, noise, mean,
                                    variance, rotation_tolerance_degrees);
}

TEST_F(HybridRotationAveragingTest, FiveHundredViewsTestWithLargeNoise) {
  const int num_views = 500;
  const int num_view_pairs = 2000;
  const double noise = 1.0;
  const double mean = 0.0;
  const double variance = 0.5;
  const double rotation_tolerance_degrees = 4.0;
  TestHybridRotationEstimator(num_views, num_view_pairs, noise, mean,
                                    variance, rotation_tolerance_degrees);
}

TEST_F(HybridRotationAveragingTest, OneThousandViewsTestWithSmallNoise) {
  const int num_views = 1000;
  const int num_view_pairs = 4000;
  const double noise = 1.0;
  const double mean = 0.0;
  const double variance = 0.2;
  const double rotation_tolerance_degrees = 4.0;
  TestHybridRotationEstimator(num_views, num_view_pairs, noise, mean,
                                    variance, rotation_tolerance_degrees);
}

TEST_F(HybridRotationAveragingTest, OneThousandViewsTestWithLargeNoise) {
  const int num_views = 1000;
  const int num_view_pairs = 4000;
  const double noise = 1.0;
  const double mean = 0.0;
  const double variance = 0.5;
  const double rotation_tolerance_degrees = 4.0;
  TestHybridRotationEstimator(num_views, num_view_pairs, noise, mean,
                                    variance, rotation_tolerance_degrees);
}

TEST_F(HybridRotationAveragingTest, FiveThousandViewsTestWithSmallNoise) {
  const int num_views = 5000;
  const int num_view_pairs = 20000;
  const double noise = 1.0;
  const double mean = 0.0;
  const double variance = 0.2;
  const double rotation_tolerance_degrees = 4.0;
  TestHybridRotationEstimator(num_views, num_view_pairs, noise, mean,
                                    variance, rotation_tolerance_degrees);
}

TEST_F(HybridRotationAveragingTest, FiveThousandViewsTestWithLargeNoise) {
  const int num_views = 5000;
  const int num_view_pairs = 20000;
  const double noise = 1.0;
  const double mean = 0.0;
  const double variance = 0.5;
  const double rotation_tolerance_degrees = 4.0;
  TestHybridRotationEstimator(num_views, num_view_pairs, noise, mean,
                                    variance, rotation_tolerance_degrees);
}

}  // namespace gopt
