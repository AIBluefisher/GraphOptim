#ifndef GRAPH_NODE_H_
#define GRAPH_NODE_H_

#include <iostream>
#include <limits>

#include <Eigen/Core>

namespace gopt {
namespace graph {

typedef size_t node_t;
const node_t kInvalidNodeId = std::numeric_limits<node_t>::max();

struct Node {
  node_t id = kInvalidNodeId;

  Node() {}

  Node(node_t idx) { id = idx; }

  Node(const Node& node) { id = node.id; }

  bool operator==(const Node& node) { return id == node.id; }

  // sort in ascending order
  static bool CompareById(const Node& node1, const Node& node2) {
    return node1.id < node2.id;
  }
};

struct ImageNode : Node {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Absolute rotation of the image node.
  Eigen::Vector3d rotation = Eigen::Vector3d::Zero();

  // Absolute translation of the image node.
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  std::string img_path = "";

  ImageNode()
  : Node(kInvalidNodeId) {
    img_path = "";
  }

  ImageNode(const node_t& idx, const std::string& path = "") : Node(idx) {
    img_path = path;
  }

  ImageNode(const ImageNode& img_node) {
    id = img_node.id;
    img_path = img_node.img_path;
  }
};
using ViewNode = ImageNode;

}  // namespace graph
}  // namespace gopt

#endif  // GRAPH_NODE_H_
