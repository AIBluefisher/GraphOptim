#ifndef GRAPH_EDGE_H_
#define GRAPH_EDGE_H_

#include <iostream>
#include <limits>

#include <Eigen/Core>

namespace gopt {
namespace graph {

typedef double weight_d;
const weight_d kDefaultWeight = 1.0;
const weight_d kInvalidWeight = std::numeric_limits<weight_d>::max();

struct Edge {
  node_t src = kInvalidNodeId;
  node_t dst = kInvalidNodeId;
  weight_d weight = kDefaultWeight;

  Edge() {}

  Edge(node_t i, node_t j) {
    src = i;
    dst = j;
    weight = kDefaultWeight;
  }

  Edge(node_t i, node_t j, weight_d w) {
    src = i;
    dst = j;
    weight = w;
  }

  Edge(const Edge& edge) {
    src = edge.src;
    dst = edge.dst;
    weight = edge.weight;
  }
};

template <typename EdgeType>
struct CmpAscent {
  bool operator()(const EdgeType& edge1, const EdgeType& edge2) {
    return edge1.weight > edge2.weight;
  }
};

template <typename EdgeType>
struct CmpDescent {
  bool operator()(const EdgeType& edge1, const EdgeType& edge2) {
    return edge1.weight < edge2.weight;
  }
};

template <typename EdgeType>
using LargerEdgePriorityQueue =
std::priority_queue<EdgeType, std::vector<EdgeType>, CmpDescent<EdgeType>>;

template <typename EdgeType>
using SmallerEdgePriorityQueue =
std::priority_queue<EdgeType, std::vector<EdgeType>, CmpAscent<EdgeType>>;

struct ViewEdge : Edge {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d rotation_2;

  Eigen::Vector3d translation_2;  
};

}  // namespace graph
}  // namespace gopt

#endif  // GRAPH_EDGE_H_
