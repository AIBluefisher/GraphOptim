#ifndef GRAPH_UNION_FIND_H_
#define GRAPH_UNION_FIND_H_

#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace gopt {
namespace graph {

class UnionFind {
 public:
  UnionFind() {}
  UnionFind(size_t n);

  // union find operations.
  void Init(size_t n);
  void InitWithNodes(const std::vector<size_t>& nodes);
  size_t FindRoot(size_t x);
  void Union(size_t x, size_t y);

  // Get functions.
  std::vector<size_t> GetRanks() const;
  std::vector<size_t> GetParents() const;
  // Components are the unique ids of parents.
  std::unordered_set<size_t> GetConnectedComponents() const;

 private:
  std::vector<size_t> ranks_;
  std::vector<size_t> parents_;
  std::vector<size_t> nodes_;
  std::unordered_map<size_t, size_t> nodes_mapper_;
};

}  // namespace graph
}  // namespace gopt

#endif  // GRAPH_UNION_FIND_H_