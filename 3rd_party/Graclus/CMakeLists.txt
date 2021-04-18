include_directories(
  .
)

file(GLOB GRACLUS_HEADER_FILES "metisLib/*.h")
file(GLOB GRACLUS_SOURCE_FILES "metisLib/*.c")

OPTIMIZER_ADD_LIBRARY(graclus
  ${GRACLUS_HEADER_FILES}
  ${GRACLUS_SOURCE_FILES}
  multilevelLib/wkkm.c
  multilevelLib/mlkkm.c
)
