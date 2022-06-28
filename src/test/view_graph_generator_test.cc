#include "test/view_graph_generator.h"

#include "gtest/gtest.h"

namespace gopt {

TEST(VIEW_GRAPH_GENERATOR_TEST, TEST_GENERATE_SINGLE_VIEW_GRAPH) {
  ViewGraphGenerator::ViewGraphGeneratorOptions options;
  options.num_scenes = 1;
  ViewGraphGenerator generator(options);
  generator.Generate("./");
}

}  // namespace gopt
