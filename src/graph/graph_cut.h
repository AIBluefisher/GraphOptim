#ifndef GRAPH_GRAPH_CUT_H_
#define GRAPH_GRAPH_CUT_H_

#include <glog/logging.h>

#include <unordered_map>
#include <vector>

namespace gopt {
namespace graph {

// Compute the normalized min-cut of an undirected graph using Graclus.
// Partitions the graph into clusters and returns the cluster labels per vertex.
std::unordered_map<int, int> ComputeNormalizedMinGraphCut(
    const std::vector<std::pair<int, int>>& edges,
    const std::vector<int>& weights, const int num_parts);

}  // namespace graph
}  // namespace gopt

#endif  // GRAPH_GRAPH_CUT_H_
