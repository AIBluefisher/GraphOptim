cmake_minimum_required(VERSION 3.5)
project(graph_optimizer)

set(GRAPH_OPTIMIZER_VERSION 1.0)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

include(CheckCXXCompilerFlag)

# Include helper macros and commands, and allow the included file to override
# the CMake policies in this file
message("Cmake module path: ${CMAKE_CURRENT_SOURCE_DIR}")
# include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/CMakeHelper.cmake NO_POLICY_SCOPE)

option(OPENMP_ENABLED "Whether to enable OpenMP parallelization" ON)
option(TESTS_ENABLED "Whether to enable Unit Test" ON)

# Finding packages.
if(OPENMP_ENABLED)
  find_package(OpenMP QUIET)
  if(OPENMP_ENABLED AND OPENMP_FOUND)
    message(STATUS "Enabling OpenMP support")
    add_definitions("-DOPENMP_ENABLED")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  else()
    message(STATUS "Disabling OpenMP support")
  endif()
endif()

find_package(Ceres REQUIRED)

find_package(Eigen3 REQUIRED)
# Use a larger inlining threshold for Clang, since it hobbles Eigen,
# resulting in an unreasonably slow version of the blas routines. The
# -Qunused-arguments is needed because CMake passes the inline
# threshold to the linker and clang complains about it and dies.
if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  set(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -Qunused-arguments -mllvm -inline-threshold=600")
  # Older versions of Clang (<= 2.9) do not support the 'return-type-c-linkage'
  # option, so check for its presence before adding it to the default flags set.
  include(CheckCXXCompilerFlag)
  check_cxx_compiler_flag("-Wno-return-type-c-linkage"
    HAVE_RETURN_TYPE_C_LINKAGE)
  if (HAVE_RETURN_TYPE_C_LINKAGE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-return-type-c-linkage")
  endif(HAVE_RETURN_TYPE_C_LINKAGE)
endif ()

find_package(Glog REQUIRED)
find_package(GTest REQUIRED)
if (GTEST_FOUND)
   message("Found GTest: ${GTEST_INCLUDE_DIRS}")
   set(GTEST_LIBRARIES ${GTEST_MAIN_LIBRARY} ${GTEST_LIBRARY})
   message("Linked Libraries: ${GTEST_MAIN_LIBRARY} ${GTEST_LIBRARY}")
else (GTEST_FOUND)
   message(FATAL_ERROR "Can't find GTest. Please set GTEST_INCLUDE_DIRS & GTEST_LIBRARIES")
endif (GTEST_FOUND)

find_package(GFlags REQUIRED)
if(GFLAGS_FOUND)
  message("Found GFlags: ${GFLAGS_INCLUDE_DIRS} ${GFLAGS_LIBRARIES}")
else(GFLAGS_FOUND)
  message(FATAL ERROR "Can't find GFlags. Please set GFLAGS_INCLUDE_DIR & GFLAGS_LIBRARY")
endif(GFLAGS_FOUND)

find_package(SuiteSparse REQUIRED)
if(SUITESPARSE_FOUND)
  # On Ubuntu the system install of SuiteSparse (v3.4.0) up to at least
  # Ubuntu 13.10 cannot be used to link shared libraries.
  if(BUILD_SHARED_LIBS AND
      SUITESPARSE_IS_BROKEN_SHARED_LINKING_UBUNTU_SYSTEM_VERSION)
    message(FATAL_ERROR "You are attempting to build graph_optimizer as a shared "
      "library on Ubuntu using a system package install of SuiteSparse "
      "3.4.0. This package is broken and does not support the "
      "construction of shared libraries (you can still build Theia as "
      "a static library).  If you wish to build a shared version of graph_optimizer "
      "you should uninstall the system install of SuiteSparse "
      "(libsuitesparse-dev) and perform a source install of SuiteSparse "
      "(we recommend that you use the latest version)")
  endif (BUILD_SHARED_LIBS AND
    SUITESPARSE_IS_BROKEN_SHARED_LINKING_UBUNTU_SYSTEM_VERSION)
  message("-- Found SuiteSparse ${SUITESPARSE_VERSION}")
  add_definitions(-DGRAPH_OPTIMIZER_SUITESPARSE_VERSION="${SUITESPARSE_VERSION}")
else(SUITESPARSE_FOUND)
  # Disable use of SuiteSparse if it cannot be found and continue.
  message(FATAL_ERROR "Can't find SuiteSparse. This library is required "
    "for bundle adjustment and for solving convex optimization problems. "
    "Please set SUITESPARSE_INCLUDE_DIR & SUITESPARSE_LIBRARY")
endif(SUITESPARSE_FOUND)

if(TESTS_ENABLED)
    enable_testing()
endif()

# Set include directories.
set(GRAPH_OPTIMIZER_INCLUDE_DIRS
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${SUITESPARSE_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
)

include_directories(
  3rd_party
  src
  ${GRAPH_OPTIMIZER_INCLUDE_DIRS}
)

# Set libraries.
set(GRAPH_OPTIMIZER_EXTERNAL_LIBRARIES
  ${GLOG_LIBRARIES}
  ${GTEST_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${CERES_LIBRARIES}
  graclus)

include(${PROJECT_SOURCE_DIR}/cmake/CMakeHelper.cmake)

add_subdirectory(3rd_party)
add_subdirectory(src)
add_subdirectory(examples)
