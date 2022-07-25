# BSD 3-Clause License

# Copyright (c) 2022, Chenyu
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

cmake_minimum_required(VERSION 3.5)
project(graph_optimizer)

set(GOPT_VERSION 1.0)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

include(CheckCXXCompilerFlag)

# Include helper macros and commands, and allow the included file to override
# the CMake policies in this file
message("Cmake module path: ${CMAKE_CURRENT_SOURCE_DIR}")

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
  add_definitions(-DGOPT_SUITESPARSE_VERSION="${SUITESPARSE_VERSION}")
else(SUITESPARSE_FOUND)
  # Disable use of SuiteSparse if it cannot be found and continue.
  message(FATAL_ERROR "Can't find SuiteSparse. This library is required "
    "for bundle adjustment and for solving convex optimization problems. "
    "Please set SUITESPARSE_INCLUDE_DIR & SUITESPARSE_LIBRARY")
endif(SUITESPARSE_FOUND)

# or to require a specific version: find_package(COLMAP 3.4 REQUIRED)
find_package(COLMAP QUIET)
if(COLMAP_FOUND)
  message(STATUS "COLMAP ENABLED")
  add_definitions(-DCOLMAP_ENABLED)
endif(COLMAP_FOUND)

if(TESTS_ENABLED)
    enable_testing()
endif()

# Set include directories.
set(GOPT_INCLUDE_DIRS
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${SUITESPARSE_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
)

include_directories(
  3rd_party
  src
  ${GOPT_INCLUDE_DIRS}
)

# Set libraries.
set(GOPT_INTERNAL_LIBRARIES
    graclus)

set(GOPT_EXTERNAL_LIBRARIES
  ${GLOG_LIBRARIES}
  ${GTEST_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${CERES_LIBRARIES}
  )

include(${PROJECT_SOURCE_DIR}/cmake/CMakeHelper.cmake)

add_subdirectory(3rd_party)
add_subdirectory(src)
add_subdirectory(examples)


################################################################################
# Install and uninstall scripts
################################################################################

# Install header files.
install(DIRECTORY src/
        DESTINATION include/gopt
        FILES_MATCHING REGEX ".*[.]h|.*[.]inl")
install(DIRECTORY 3rd_party/
        DESTINATION include/gopt/3rd_party
        FILES_MATCHING REGEX ".*[.]h|.*[.]hpp|.*[.]inl")

# Install static libraries.
install(DIRECTORY ${CMAKE_SOURCE_DIR}/lib/
        DESTINATION lib/gopt
        FILES_MATCHING PATTERN "*.a")

# Generate and install CMake configuration.
configure_file("${CMAKE_SOURCE_DIR}/cmake/CMakeConfig.cmake.in"
               "${CMAKE_CURRENT_BINARY_DIR}/GoptConfig.cmake" @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/GoptConfig.cmake"
        DESTINATION "share/gopt")

# Install find_package scripts for dependencies
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/cmake
        DESTINATION share/gopt
        FILES_MATCHING PATTERN "Find*.cmake")
