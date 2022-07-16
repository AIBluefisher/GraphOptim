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

#include "utils/types.h"

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

  const NodeType& GetNode(node_t idx) const;
  NodeType& GetNode(node_t idx);

  const std::unordered_map<node_t, NodeType>& GetNodes() const;
  std::unordered_map<node_t, NodeType>& GetNodes();
  node_t GetNodesNum() const;
  bool HasNode(const node_t& idx) const;
  bool AddNode(const NodeType& node);
  bool DeleteNode(const node_t& idx);
  bool RemoveSingletonNodes();
  std::vector<NodeType> FindSingletonNodes();
  node_t FindLeafNode(const std::unordered_map<node_t, node_t>& degrees) const;

  const std::unordered_map<node_t, EdgeMap>& GetEdges() const;
  std::unordered_map<node_t, EdgeMap>& GetEdges();
  std::unordered_map<ImagePair, EdgeType> GetAllEdgePairs() const;

  const EdgeType& GetEdge(node_t src, node_t dst) const;
  EdgeType& GetEdge(node_t src, node_t dst);

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
