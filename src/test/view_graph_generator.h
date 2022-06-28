#ifndef TEST_VIEW_GRAPH_GENERATOR_H_
#define TEST_VIEW_GRAPH_GENERATOR_H_

#include <string>
#include <unordered_map>

#include "util/random.h"
#include "graph/view_graph.h"

namespace gopt {

class ViewGraphGenerator {
 public:
  struct ViewGraphGeneratorOptions {
    // Directory to store the generated view graph.
    std::string output_path = "./";

    // Maximum of threads that used to concurrently generate the scene graphs.
    size_t num_threads = -1;

    // How many view graph to generate.
    size_t num_scenes = 1250;

    // Minimum number of nodes for a view graph.
    size_t min_num_nodes = 400;

    // Factor to expand the number of nodes.
    size_t nodes_exp_ratio = 600;

    // Edges coverage for a view graph.
    double completeness_factor = 0.25;

    // Noise factor for relative rotations.
    double rotation_sigma_factor = 15.0;

    // Outlier ratio for relative rotations (slightly higher than the
    // outlier ratio in NeuRoRA which is 0.1).
    double rotation_outlier_factor = 0.12;

    // The threshold to indicate if a relative rotation is an outlier.
    double rotation_outlier_threshold = M_PI / 9.0;

    // Noise factor for relative translations.
    double translation_sigma_factor = 20.0;

    // Outlier ratio for relative translations.
    double translation_outlier_factor = 0.4;

    // The threshold to indicate if a relative translation is an outlier.
    double translation_outlier_threshold = M_PI / 18.0;
  };

  ViewGraphGenerator(const ViewGraphGeneratorOptions& options);
  
  void Run();
  void Generate(const size_t scene_idx);
  
  graph::ViewGraph GenerateRandomGraph(
    const size_t num_nodes = 1250, const double completeness_ratio = 0.25,
    const double rotation_sigma = 15.0, const double rotation_outlier_ratio = 0.1,
    const double translation_sigma = 20.0, const double translation_outlier_ratio = 0.2,
    std::unordered_map<gopt::image_t, Eigen::Vector3d>* gt_rotations = nullptr,
    std::unordered_map<gopt::image_t, Eigen::Vector3d>* gt_positions = nullptr);
 
 private:
  void GenerateAbsolutePoses(
    const size_t num_nodes,
    std::unordered_map<gopt::image_t, Eigen::Vector3d>* gt_rotations,
    std::unordered_map<gopt::image_t, Eigen::Vector3d>* gt_positions);

  std::vector<graph::ViewEdge> GenerateRelativePoses(
    const size_t num_nodes,
    const std::unordered_map<gopt::image_t, Eigen::Vector3d>& gt_rotations,
    const std::unordered_map<gopt::image_t, Eigen::Vector3d>& gt_positions);

  void AddEdgesToViewGraph(
    const size_t num_nodes, const double completeness_ratio,
    std::vector<graph::ViewEdge> candidate_edges,
    graph::ViewGraph* view_graph);

  void PerturbViewGraphByNoise(
    const double rotation_sigma,
    const double translation_sigma,
    graph::ViewGraph* view_graph);

  void AddOutliersToViewGraph(
    const size_t num_nodes,
    const double rotation_outlier_ratio,
    const double translation_outlier_ratio,
    graph::ViewGraph* view_graph);

  void ValidateViewGraph(
    graph::ViewGraph& view_graph,
    const size_t num_nodes,
    const std::unordered_map<gopt::image_t, Eigen::Vector3d>& gt_rotations,
    const std::unordered_map<gopt::image_t, Eigen::Vector3d>& gt_positions,
    std::unordered_map<image_t, Eigen::Vector3d>* estimated_rotations,
    std::unordered_map<image_t, Eigen::Vector3d>* estimated_positions);

  void ComputeOutliers(
    const graph::ViewGraph& view_graph,
    const double rotation_threshold = M_PI / 9.0,
    const double translation_threshold = M_PI / 18.0,
    std::vector<ImagePair>* rotation_outlier_indices = nullptr,
    std::vector<ImagePair>* translation_outlier_indices = nullptr) const;

  RandomNumberGenerator rng_;

  const ViewGraphGeneratorOptions options_;
};

}  // namespace opt


#endif  // TEST_VIEW_GRAPH_GENERATOR_H_
