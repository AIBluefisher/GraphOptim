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

  bool ReadG2OFile(const std::string &filename);

 private:
  void ViewEdgesToViewPairs(
    std::unordered_map<ImagePair, TwoViewGeometry>* view_pairs);

  void InitializeGlobalRotations(
      const RotationEstimatorOptions& options,
      std::unordered_map<image_t, Eigen::Vector3d>* global_rotations);
  void InitializeGlobalRotationsRandomly(
      std::unordered_map<image_t, Eigen::Vector3d>* global_rotations);
  void InitializeGlobalRotationsFromMST(
      std::unordered_map<image_t, Eigen::Vector3d>* global_rotations);

  void InitializeGlobalPositions(
      std::unordered_map<image_t, Eigen::Vector3d>* positions);
  
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
