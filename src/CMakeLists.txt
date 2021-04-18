add_subdirectory(geometry)
add_subdirectory(graph)
add_subdirectory(math)
add_subdirectory(solver)
add_subdirectory(rotation_averaging)
add_subdirectory(util)
add_subdirectory(translation_averaging)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

OPTIMIZER_ADD_LIBRARY(${GRAPH_OPTIMIZER_LIB}
  ${GRAPH_OPTIMIZER_HEADERS}
  ${GRAPH_OPTIMIZER_SOURCES})
target_link_libraries(${GRAPH_OPTIMIZER_LIB} ${GRAPH_OPTIMIZER_EXTERNAL_LIBRARIES})
