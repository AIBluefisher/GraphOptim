#ifndef GRAPH_GRAPH_H_
#define GRAPH_GRAPH_H_

#include <queue>
#include <string>
#include <unordered_set>
#include <unordered_map>
#include <utility>
#include <vector>

#include "graph/node.h"
#include "graph/edge.h"

namespace gopt {
namespace graph {

template <typename NodeType, typename EdgeType>
class Graph {
 public:
  // Adjacent nodes and corresponding edges's info.
  typedef std::unordered_map<node_t, EdgeType> EdgeMap;

  // constructors
  Graph();
  Graph(const size_t n);
  Graph(const Graph<NodeType, EdgeType>& graph);

  // Clone operation
  Graph<NodeType, EdgeType> Clone() const;

  // graph size (equals to size of nodes)
  size_t GetSize() const;

  // Node operation
  NodeType GetNode(node_t idx) const;
  const std::unordered_map<node_t, NodeType>& GetNodes() const;
  node_t GetNodesNum() const;
  bool HasNode(const node_t& idx) const;
  bool AddNode(const NodeType& node);
  bool DeleteNode(const node_t& idx);
  bool RemoveSingletonNodes();
  std::vector<NodeType> FindSingletonNodes();
  node_t FindLeafNode(const std::unordered_map<node_t, node_t>& degrees) const;

  // Edge operation
  const std::unordered_map<node_t, EdgeMap>& GetEdges() const;
  EdgeType GetEdge(node_t src, node_t dst) const;
  size_t GetEdgesNum() const;
  bool HasEdge(const node_t& src, const node_t& dst) const;
  bool AddEdge(const EdgeType& edge);
  bool AddUEdge(const EdgeType& edge, const EdgeType& rev_edge);
  bool AlterEdge(const EdgeType& edge);
  bool DeleteEdge(const node_t& src, const node_t& dst);
  SmallerEdgePriorityQueue<EdgeType> CollectEdges() const;
  EdgeType FindConnectedEdge(const node_t& idx) const;

  // Degree operation
  void CountDegrees();
  const std::unordered_map<node_t, node_t>& GetDegrees() const;
  void CountOutDegrees();
  const std::unordered_map<node_t, node_t>& GetOutDegrees() const;
  void CountInDegrees();
  const std::unordered_map<node_t, node_t>& GetInDegrees() const;

  // Minimum Spanning Tree (MST) algorithm
  std::vector<EdgeType> Kruskal() const;

  // breadth-first-search algorithm
  std::vector<EdgeType> ShortestPath(const node_t& src,
                                     const node_t& dst) const;

  // Graph-cut algorithm
  std::unordered_map<int, int> NormalizedCut(const size_t cluster_num) const;

  // graph information presentation.
  void ShowInfo() const;
  void ShowInfo(const std::string& filename) const;
  void OutputSVG(const std::string& filename) const;

  Graph<NodeType, EdgeType> ExtractLargestCC() const;

  std::unordered_map<node_t, std::unordered_set<node_t>>
  ExtractConnectedComponents() const;

  std::vector<NodeType> ToStdVectorNodes() const;

 protected:
  size_t size_;
  std::unordered_map<node_t, NodeType> nodes_;
  std::unordered_map<node_t, EdgeMap> edges_;
  // degree of nodes: node_id, degree
  std::unordered_map<node_t, node_t> degrees_;
  std::unordered_map<node_t, node_t> out_degrees_;
  std::unordered_map<node_t, node_t> in_degrees_;
};

}  // namespace graph
}  // namespace gopt

#include "graph/graph.inl"

#endif  // GRAPH_GRAPH_H_
