#include "graph_cut.h"

#include "gtest/gtest.h"

namespace gopt {
namespace graph {

TEST(GRAPH_CUT_TEST, TestComputeNormalizedMinGraphCut) {
  const std::vector<std::pair<int, int>> edges = {
      {3, 4}, {3, 6}, {3, 5}, {0, 4}, {0, 1}, {0, 6}, {0, 7}, {0, 5},
      {0, 2}, {4, 1}, {1, 6}, {1, 5}, {6, 7}, {7, 5}, {5, 2}, {3, 4}};
  const std::vector<int> weights = {0, 3, 1, 3,  1, 2, 6, 1,
                                    8, 1, 1, 80, 2, 1, 1, 4};
  const auto cut_labels = ComputeNormalizedMinGraphCut(edges, weights, 2);
  EXPECT_EQ(cut_labels.size(), 8);
  for (const auto& label : cut_labels) {
    EXPECT_GE(label.second, 0);
    EXPECT_LT(label.second, 2);
  }
}

TEST(GRAPH_CUT_TEST, TestComputeNormalizedMinGraphCutDuplicateEdge) {
  const std::vector<std::pair<int, int>> edges = {
      {3, 4}, {3, 6}, {3, 5}, {0, 4}, {0, 1}, {0, 6}, {0, 7}, {0, 5}, {0, 2},
      {4, 1}, {1, 6}, {1, 5}, {6, 7}, {7, 5}, {5, 2}, {3, 4}, {3, 4}};
  const std::vector<int> weights = {0, 3, 1,  3, 1, 2, 6, 1, 8,
                                    1, 1, 80, 2, 1, 1, 4, 4};
  const auto cut_labels = ComputeNormalizedMinGraphCut(edges, weights, 2);
  EXPECT_EQ(cut_labels.size(), 8);
  for (const auto& label : cut_labels) {
    EXPECT_GE(label.second, 0);
    EXPECT_LT(label.second, 2);
  }
}

TEST(GRAPH_CUT_TEST, TestComputeNormalizedMinGraphCutMissingVertex) {
  const std::vector<std::pair<int, int>> edges = {
      {3, 4}, {3, 6}, {3, 5}, {0, 1}, {0, 6}, {0, 7}, {0, 5},
      {0, 2}, {4, 1}, {1, 6}, {1, 5}, {6, 7}, {7, 5}, {5, 2}};
  const std::vector<int> weights = {0, 3, 1, 3, 1, 2, 6, 1, 8, 1, 1, 80, 2, 1};
  const auto cut_labels = ComputeNormalizedMinGraphCut(edges, weights, 2);
  EXPECT_EQ(cut_labels.size(), 8);
  for (const auto& label : cut_labels) {
    EXPECT_GE(label.second, 0);
    EXPECT_LT(label.second, 2);
  }
}

TEST(GRAPH_CUT_TEST, TestComputeNormalizedMinGraphCutDisconnected) {
  const std::vector<std::pair<int, int>> edges = {{0, 1}, {1, 2}, {3, 4}};
  const std::vector<int> weights = {1, 3, 1};
  const auto cut_labels = ComputeNormalizedMinGraphCut(edges, weights, 2);
  EXPECT_EQ(cut_labels.size(), 5);
  for (const auto& label : cut_labels) {
    EXPECT_GE(label.second, 0);
    EXPECT_LT(label.second, 2);
  }
}

}  // namespace graph
}  // namespace gopt
