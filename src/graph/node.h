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
  
  // Absolute path of image.
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
    rotation = img_node.rotation;
    translation = img_node.translation;
  }

  // ImageNode& operator=(const ImageNode& img_node) = delete;
};
using ViewNode = ImageNode;

}  // namespace graph
}  // namespace gopt

#endif  // GRAPH_NODE_H_
