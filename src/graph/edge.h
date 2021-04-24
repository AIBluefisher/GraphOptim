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
