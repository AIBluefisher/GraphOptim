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

#ifndef GRAPH_VIEW_GRAPH_H_
#define GRAPH_VIEW_GRAPH_H_

#include <memory>
#include <unordered_map>

#include "graph/graph.h"
#include "graph/node.h"
#include "graph/edge.h"

#include "rotation_averaging/rotation_estimator.h"
#include "translation_averaging/position_estimator.h"

namespace gopt {
namespace graph {

class ViewGraph : public Graph<ViewNode, ViewEdge> {
 public:
  struct ViewGraphOptions {

  };

  ViewGraph();

  bool MotionAveraging(
      const RotationEstimatorOptions& rotation_estimator_options,
      const PositionEstimatorOptions& position_estimator_options,
      std::unordered_map<image_t, Eigen::Vector3d>* global_rotations,
      std::unordered_map<image_t, Eigen::Vector3d>* positions);

  bool RotationAveraging(
      const RotationEstimatorOptions& options,
      std::unordered_map<image_t, Eigen::Vector3d>* global_rotations);

  bool TranslationAveraging(
      const PositionEstimatorOptions& options,
      std::unordered_map<image_t, Eigen::Vector3d>* positions);

  void InitializeGlobalRotationsRandomly(
      std::unordered_map<image_t, Eigen::Vector3d>* global_rotations);
  void InitializeGlobalRotationsFromMST(
      std::unordered_map<image_t, Eigen::Vector3d>* global_rotations);
  
  void InitializeGlobalRotations(
      const RotationEstimatorOptions& options,
      std::unordered_map<image_t, Eigen::Vector3d>* global_rotations);

  void InitializeGlobalPositions(
      std::unordered_map<image_t, Eigen::Vector3d>* positions);

  bool ReadG2OFile(const std::string& filename);
  void WriteG2OFile(const std::string& filename);

 private:
  void ViewEdgesToViewPairs(
    std::unordered_map<ImagePair, TwoViewGeometry>* view_pairs);
  
  std::unique_ptr<RotationEstimator> CreateRotationEstimator(
      const RotationEstimatorOptions& options);

  std::unique_ptr<PositionEstimator> CreatePositionEstimator(
      const PositionEstimatorOptions& options);

  ViewGraphOptions options_;
};
// A view graph conceptually is equivalents to a pose graph.
using PoseGraph = ViewGraph;

}  // namespace graph
}  // namespace gopt

#endif  // GRAPH_VIEW_GRAPH_H_
