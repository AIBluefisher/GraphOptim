#include "graph/graph.h"

#include <glog/logging.h>
#include <fstream>

#include "graph/color_gradient.h"
#include "graph/graph_cut.h"
#include "graph/union_find.h"
#include "graph/svg_drawer.h"

namespace gopt {
namespace graph {

template <typename NodeType, typename EdgeType>
Graph<NodeType, EdgeType>::Graph() {
  size_ = 0;
}

template <typename NodeType, typename EdgeType>
Graph<NodeType, EdgeType>::Graph(const size_t n) {
  size_ = n;
  for (size_t i = 0; i < n; i++) {
    degrees_[i] = 0;
  }
}

template <typename NodeType, typename EdgeType>
Graph<NodeType, EdgeType>::Graph(const Graph<NodeType, EdgeType>& graph) {
  nodes_ = graph.GetNodes();
  edges_ = graph.GetEdges();
  degrees_ = graph.GetDegrees();
  out_degrees_ = graph.GetOutDegrees();
  in_degrees_ = graph.GetInDegrees();
}

template <typename NodeType, typename EdgeType>
Graph<NodeType, EdgeType> Graph<NodeType, EdgeType>::Clone() const {
  Graph<NodeType, EdgeType> graph(this->size_);

  for (const auto& node_iter : nodes_) {
    graph.AddNode(node_iter.second);
  }

  for (const auto& edge_iter : edges_) {
    const EdgeMap& em = edge_iter.second;
    for (const auto& em_iter : em) {
      graph.AddEdge(em_iter.second);
    }
  }
  return graph;
}

template <typename NodeType, typename EdgeType>
NodeType Graph<NodeType, EdgeType>::GetNode(node_t idx) const {
  CHECK(idx != kInvalidNodeId);

  if (!HasNode(idx)) {
    return NodeType();
  }

  return nodes_.at(idx);
}

template <typename NodeType, typename EdgeType>
const std::unordered_map<node_t, NodeType>&
Graph<NodeType, EdgeType>::GetNodes() const {
  return nodes_;
}

template <typename NodeType, typename EdgeType>
size_t Graph<NodeType, EdgeType>::GetNodesNum() const {
  return nodes_.size();
}

template <typename NodeType, typename EdgeType>
bool Graph<NodeType, EdgeType>::HasNode(const node_t& idx) const {
  if (nodes_.find(idx) == nodes_.end()) {
    return false;
  }
  return true;
}

template <typename NodeType, typename EdgeType>
bool Graph<NodeType, EdgeType>::AddNode(const NodeType& node) {
  if (HasNode(node.id)) {
    return false;
  }

  size_++;

  nodes_[node.id] = node;
  return true;
}

template <typename NodeType, typename EdgeType>
bool Graph<NodeType, EdgeType>::DeleteNode(const node_t& idx) {
  if (!HasNode(idx)) {
    return false;
  }

  size_--;
  nodes_.erase(idx);
  return true;
}

template <typename NodeType, typename EdgeType>
bool Graph<NodeType, EdgeType>::RemoveSingletonNodes() {
  // this->CountDegrees();
  for (auto it = degrees_.begin(); it != degrees_.end(); ++it) {
    if (it->second == 0) {
      nodes_.erase(it->first);
      degrees_.erase(it->first);
    }
  }
  return true;
}

template <typename NodeType, typename EdgeType>
std::vector<NodeType> Graph<NodeType, EdgeType>::FindSingletonNodes() {
  std::vector<NodeType> singleton_nodes;
  this->CountOutDegrees();
  this->CountInDegrees();
  this->CountDegrees();
  for (auto it = degrees_.begin(); it != degrees_.end(); ++it) {
    if (it->second == 0) {
      singleton_nodes.push_back(this->GetNode(it->first));
    }
  }
  return singleton_nodes;
}

template <typename NodeType, typename EdgeType>
const std::unordered_map<node_t, std::unordered_map<node_t, EdgeType>>&
Graph<NodeType, EdgeType>::GetEdges() const {
  return edges_;
}

template <typename NodeType, typename EdgeType>
EdgeType Graph<NodeType, EdgeType>::GetEdge(node_t src, node_t dst) const {
  CHECK(src != kInvalidNodeId);
  CHECK(dst != kInvalidNodeId);

  if (!HasEdge(src, dst)) {
    return EdgeType();
  }

  return edges_.at(src).at(dst);
}

template <typename NodeType, typename EdgeType>
size_t Graph<NodeType, EdgeType>::GetEdgesNum() const {
  size_t sum = 0;
  for (const auto& edge_iter : edges_) {
    const EdgeMap& em = edge_iter.second;
    sum += em.size();
  }

  return sum;
}

template <typename NodeType, typename EdgeType>
bool Graph<NodeType, EdgeType>::HasEdge(
    const node_t& src, const node_t& dst) const {
  CHECK(src != kInvalidNodeId);
  CHECK(dst != kInvalidNodeId);

  const auto& em_ite = edges_.find(src);
  if (em_ite == edges_.end()) {
    return false;
  }

  const EdgeMap& em = em_ite->second;
  if (em.find(dst) == em.end()) {
    return false;
  }
  return true;
}

template <typename NodeType, typename EdgeType>
bool Graph<NodeType, EdgeType>::AddEdge(const EdgeType& edge) {
  CHECK(edge.src != kInvalidNodeId);
  CHECK(edge.dst != kInvalidNodeId);

  if (HasEdge(edge.src, edge.dst)) {
    return false;
  }

  if (!HasNode(edge.src)) {
    this->AddNode(edge.src);
  }
  if (!HasNode(edge.dst)) {
    this->AddNode(edge.dst);
  }

  edges_[edge.src][edge.dst] = edge;

  return true;
}

template <typename NodeType, typename EdgeType>
bool Graph<NodeType, EdgeType>::AlterEdge(const EdgeType& edge) {
  CHECK(edge.src != kInvalidNodeId);
  CHECK(edge.dst != kInvalidNodeId);

  if (!HasEdge(edge.src, edge.dst)) {
    return false;
  }

  if (!HasNode(edge.src) || !HasNode(edge.dst)) {
    return false;
  }

  edges_.at(edge.src).at(edge.dst) = edge;
  return true;
}

template <typename NodeType, typename EdgeType>
bool Graph<NodeType, EdgeType>::AddUEdge(const EdgeType& edge,
                                         const EdgeType& rev_edge) {
  return this->AddEdge(edge) && this->AddEdge(rev_edge);
}

template <typename NodeType, typename EdgeType>
bool Graph<NodeType, EdgeType>::DeleteEdge(
    const node_t& src,  const node_t& dst) {
  CHECK(src != kInvalidNodeId);
  CHECK(dst != kInvalidNodeId);

  if (!HasEdge(src, dst)) {
    return false;
  }

  auto em_ite = edges_.find(src);
  em_ite->second.erase(em_ite->second.find(dst));

  if (edges_[src].empty()) {
    edges_.erase(em_ite);
  }
  return true;
}

template <typename NodeType, typename EdgeType>
EdgeType Graph<NodeType, EdgeType>::FindConnectedEdge(
    const node_t& idx) const {
  EdgeType edge;
  for (auto it = edges_.begin(); it != edges_.end(); ++it) {
    auto em = it->second;
    bool find = false;
    for (auto em_it = em.begin(); em_it != em.end(); em_it++) {
      if (static_cast<node_t>(em_it->second.src) == idx ||
          static_cast<node_t>(em_it->first) == idx) {
        edge = em_it->second;
        find = true;
        break;
      }
    }
    if (find) {
      break;
    }
  }
  return edge;
}

template <typename NodeType, typename EdgeType>
size_t Graph<NodeType, EdgeType>::GetSize() const {
  CHECK_EQ(nodes_.size(), size_);

  return nodes_.size();
}

template <typename NodeType, typename EdgeType>
void Graph<NodeType, EdgeType>::CountDegrees() {
  // this->CountInDegrees();
  // this->CountOutDegrees();
  degrees_.clear();
  for (const auto& node_iter : nodes_) {
    const node_t id = node_iter.second.id;
    degrees_[id] = in_degrees_[id] + out_degrees_[id];
  }
}

template <typename NodeType, typename EdgeType>
const std::unordered_map<node_t, node_t>&
Graph<NodeType, EdgeType>::GetDegrees() const {
  return degrees_;
}

template <typename NodeType, typename EdgeType>
void Graph<NodeType, EdgeType>::CountOutDegrees() {
  out_degrees_.clear();
  for (const auto& edge_iter : edges_) {
    const EdgeMap& em = edge_iter.second;
    out_degrees_[edge_iter.first] = em.size();
  }
}

template <typename NodeType, typename EdgeType>
const std::unordered_map<node_t, node_t>&
Graph<NodeType, EdgeType>::GetOutDegrees() const {
  return out_degrees_;
}

template <typename NodeType, typename EdgeType>
void Graph<NodeType, EdgeType>::CountInDegrees() {
  in_degrees_.clear();
  // initializing degree before using
  for (const auto& node_iter : nodes_) {
    in_degrees_[node_iter.second.id] = 0;
  }

  for (const auto& edge_iter : edges_) {
    const EdgeMap& em = edge_iter.second;
    for (const auto& em_iter : em) {
      in_degrees_[em_iter.first]++;
    }
  }
}

template <typename NodeType, typename EdgeType>
const std::unordered_map<node_t, node_t>&
Graph<NodeType, EdgeType>::GetInDegrees() const {
  return out_degrees_;
}

template <typename NodeType, typename EdgeType>
node_t Graph<NodeType, EdgeType>::FindLeafNode(
    const std::unordered_map<node_t, node_t>& degrees) const {
  // Finding nodes with degree equals 1
  node_t idx = kInvalidNodeId;
  for (const auto& deg_iter : degrees) {
    if (deg_iter.second == 1) {
      idx = deg_iter.first;
      break;
    }
  }
  return idx;
}

template <typename NodeType, typename EdgeType>
std::vector<EdgeType> Graph<NodeType, EdgeType>::Kruskal() const {
  std::vector<EdgeType> mst_edges;
  SmallerEdgePriorityQueue<EdgeType> edges = this->CollectEdges();

  std::vector<node_t> nodes;
  nodes.reserve(nodes_.size());
  for (const auto& node_iter : nodes_) {
    nodes.push_back(node_iter.second.id);
  }
  std::sort(nodes.begin(), nodes.end());

  UnionFind union_find(nodes_.size());
  union_find.InitWithNodes(nodes);

  while (!edges.empty()) {
    const EdgeType edge = edges.top();
    edges.pop();
    size_t src = edge.src, dst = edge.dst;
    if (union_find.FindRoot(src) != union_find.FindRoot(dst)) {
      mst_edges.emplace_back(edge);
      union_find.Union(src, dst);
    }
  }

  return mst_edges;
}

template <typename NodeType, typename EdgeType>
std::vector<EdgeType> Graph<NodeType, EdgeType>::ShortestPath(
    const node_t& src, const node_t& dst) const {
  CHECK(src != kInvalidNodeId);
  CHECK(dst != kInvalidNodeId);

  std::vector<EdgeType> paths;
  std::queue<node_t> qu;
  std::unordered_map<node_t, node_t> parents;
  std::unordered_map<node_t, bool> visited;

  const std::unordered_map<node_t, EdgeMap>& edges = edges_;

  for (const auto& node_iter : nodes_) {
    visited.insert(std::make_pair(node_iter.second.id, false));
  }

  qu.push(src);
  parents[src] = kInvalidNodeId;
  visited[src] = true;

  while (!qu.empty()) {
    const node_t cur_id = qu.front();
    qu.pop();

    if (cur_id == dst) { // arrive destination
      node_t id = cur_id;
      std::string shortest_path = "";
      shortest_path = std::to_string(id) + "->";
      while (parents[id] != kInvalidNodeId) {
        shortest_path += std::to_string(parents[id]) + "->";
        const EdgeType& edge = edges.at(parents[id]).at(id);
        paths.push_back(edge);
        id = parents[id];
      }

      shortest_path += std::to_string(dst);
      LOG(INFO) << "Shortest Path (BFS): " << shortest_path;
      break;
    }

    const EdgeMap& em = edges.at(cur_id);
    for (const auto& em_iter : em) {
      if (!visited[em_iter.first]) {
        visited[em_iter.first] = true;
        qu.push(em_iter.first);
        parents[em_iter.first] = cur_id;
      }
    }
  }
  std::reverse(paths.begin(), paths.end());

  return paths;
}

template <typename NodeType, typename EdgeType>
std::unordered_map<int, int> Graph<NodeType, EdgeType>::NormalizedCut(
    const size_t cluster_num) const {
  std::vector<std::pair<int, int>> edges;
  std::vector<int> weights;

  for (const auto& edge_iter : edges_) {
    const EdgeMap& em = edge_iter.second;
    for (auto em_iter : em) {
      edges.emplace_back(
          static_cast<int>(em_iter.second.src),
          static_cast<int>(em_iter.second.dst));
      weights.push_back(static_cast<int>(em_iter.second.weight));
    }
  }

  return ComputeNormalizedMinGraphCut(edges, weights, cluster_num);
}

template <typename NodeType, typename EdgeType>
void Graph<NodeType, EdgeType>::ShowInfo() const {
  std::vector<NodeType> nodes = this->ToStdVectorNodes();

  LOG(INFO) << "[Graph Info]\n";
  LOG(INFO) << "Total nodes: " << std::to_string(GetNodesNum());
  LOG(INFO) << "\nTotal edges: " << std::to_string(GetEdgesNum());
  LOG(INFO) << "\n[Node]: \n";
  for (uint i = 0; i < nodes.size(); i++) {
    std::cout << nodes[i].id << " ";
  }
  LOG(INFO) << "\n[Edge]: \n";
  for (const auto edge_iter : edges_) {
    const auto& em = edge_iter.second;
    if (em.size() == 0) {
      continue;
    }

    for (const auto& em_iter : em) {
      std::cout << "(" << em_iter.second.src << ", " << em_iter.second.dst
                << ") ";
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
}

template <typename NodeType, typename EdgeType>
void Graph<NodeType, EdgeType>::ShowInfo(const std::string& filename) const {
  std::ofstream out(filename);
  if (!out.is_open()) {
    std::cout << filename << " cannot be opened!\n";
    return;
  }

  std::vector<NodeType> nodes = this->ToStdVectorNodes();

  out << "[Graph Info]\n";
  out << "Total nodes: " + std::to_string(GetNodesNum());
  out << "\nTotal edges: " + std::to_string(GetEdgesNum());
  out << "\n[Node]: \n";
  for (int i = 0; i < nodes.size(); i++) {
    out << nodes[i].id << " ";
  }

  out << "\n[Edge]: \n";
  for (const auto edge_iter : edges_) {
    const auto& em = edge_iter.second;
    if (em.size() == 0) {
      continue;
    }

    for (const auto& em_iter : em) {
      out << "(" << em_iter.second.src << ", " << em_iter.second.dst << ") ";
    }
    out << std::endl;
  }
}

template <typename NodeType, typename EdgeType>
Graph<NodeType, EdgeType> Graph<NodeType, EdgeType>::ExtractLargestCC() const {
  const std::unordered_map<node_t, std::unordered_set<node_t>> components =
    ExtractConnectedComponents();

  size_t num_largest_component = 0;
  node_t largest_component_id;
  for (const auto& it : components) {
    if (num_largest_component < it.second.size()) {
      num_largest_component = it.second.size();
      largest_component_id = it.first;
    }
  }

  Graph<NodeType, EdgeType> largest_cc;
    for (const auto edge_iter : edges_) {
    const auto& em = edge_iter.second;
    for (const auto& em_iter : em) {
      if (components.at(largest_component_id).count(em_iter.second.src) == 0 ||
          components.at(largest_component_id).count(em_iter.second.dst) == 0) {
        continue;
      }
      largest_cc.AddEdge(em_iter.second);
    }
  }

  return largest_cc;
}

template <typename NodeType, typename EdgeType>
std::unordered_map<node_t, std::unordered_set<node_t>>
Graph<NodeType, EdgeType>::ExtractConnectedComponents() const {
  graph::UnionFind uf(nodes_.size());

  std::vector<node_t> node_ids;
  node_ids.reserve(nodes_.size());
  for (const auto& node_it : nodes_) {
    node_ids.push_back(node_it.first);
  }
  uf.InitWithNodes(node_ids);

  for (const auto edge_iter : edges_) {
    const auto& em = edge_iter.second;
    for (const auto& em_iter : em) {
      uf.Union(em_iter.second.src, em_iter.second.dst);
    }
  }

  std::unordered_map<node_t, std::unordered_set<node_t>> components;
  for (const node_t node_id : node_ids) {
    const node_t parent_id = uf.FindRoot(node_id);
    components[parent_id].insert(node_id);
  }

  return components;
}

template <typename NodeType, typename EdgeType>
SmallerEdgePriorityQueue<EdgeType> Graph<NodeType, EdgeType>::CollectEdges() const {
  SmallerEdgePriorityQueue<EdgeType> edges;
  for (const auto edge_iter : edges_) {
    const auto& em = edge_iter.second;
    for (const auto& em_iter : em) {
      edges.push(em_iter.second);
    }
  }
  return edges;
}

template <typename NodeType, typename EdgeType>
std::vector<NodeType> Graph<NodeType, EdgeType>::ToStdVectorNodes() const {
  std::vector<NodeType> nodes;
  for (const auto& node_iter : nodes_) {
    nodes.push_back(node_iter.second);
  }
  std::sort(nodes.begin(), nodes.end(), Node::CompareById);
  return nodes;
}

template <typename NodeType, typename EdgeType>
void Graph<NodeType, EdgeType>::OutputSVG(const std::string &filename) const {
  const float scale_factor = 5.0f;
  const size_t image_num = nodes_.size();

  SvgDrawer svg_drawer((image_num + 3) * 5, (image_num + 3) * 5);

  // Draw rectangles for all image pairs.
  for (const auto &it : edges_) {
    const size_t i = it.first;
    const EdgeMap &em = it.second;

    for (auto &em_it : em) {
      const size_t j = em_it.first;
      const float score = em_it.second.weight;

      std::ostringstream os_color;
      os_color << "rgb(" << 0 << "," << 0 << "," << 255 << ")";

      std::ostringstream os_tooltip;
      os_tooltip << "(" << j << "," << i << " " << score << ")";
      svg_drawer.DrawSquare(
          j * scale_factor, i * scale_factor, scale_factor / 2.0f,
          SvgStyle().Fill(os_color.str()).NoStroke().ToolTip(os_tooltip.str()));

      os_tooltip.clear();
      os_tooltip << "(" << i << "," << j << " " << score << ")";
      svg_drawer.DrawSquare(
          i * scale_factor, j * scale_factor, scale_factor / 2.0f,
          SvgStyle().Fill(os_color.str()).NoStroke().ToolTip(os_tooltip.str()));
    }
  }

  // Display axes with 0-> image_num annotation.
  std::ostringstream os_num_images;
  os_num_images << image_num;
  svg_drawer.DrawText((image_num + 1) * scale_factor, scale_factor,
                      scale_factor, "0", "black");
  svg_drawer.DrawText((image_num + 1) * scale_factor,
                      (image_num - 1) * scale_factor, scale_factor,
                      os_num_images.str(), "black");
  svg_drawer.DrawLine((image_num + 1) * scale_factor, 2 * scale_factor,
                      (image_num + 1) * scale_factor,
                      (image_num - 2) * scale_factor,
                      SvgStyle().Stroke("black", 1.0));
  svg_drawer.DrawText(scale_factor, (image_num + 1) * scale_factor,
                      scale_factor, "0", "black");
  svg_drawer.DrawText((image_num - 1) * scale_factor,
                      (image_num + 1) * scale_factor, scale_factor,
                      os_num_images.str(), "black");
  svg_drawer.DrawLine(2 * scale_factor, (image_num + 1) * scale_factor,
                      (image_num - 2) * scale_factor,
                      (image_num + 1) * scale_factor,
                      SvgStyle().Stroke("black", 1.0));

  std::ofstream svg_ofs(filename);
  svg_ofs << svg_drawer.CloseSvgFile().str();
}

}  // namespace graph
}  // namespace gopt
