OPTIMIZER_ADD_HEADERS(
  color_gradient.h
  edge.h
  graph_cut.h
  graph.h
  node.h
  union_find.h
  view_graph.h
  svg_drawer.h)

OPTIMIZER_ADD_SOURCES(
  graph_cut.cc
  graph.inl
  union_find.cc
  view_graph.cc)

OPTIMIZER_ADD_GTEST(union_find_test union_find_test.cc)
OPTIMIZER_ADD_GTEST(graph_test graph_test.cc)
OPTIMIZER_ADD_GTEST(graph_cut_test graph_cut_test.cc)
