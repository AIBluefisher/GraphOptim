# BSD 3-Clause License

# Copyright (c) 2021, Chenyu
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

project(GlobalSfM)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/../bin)

# or to require a specific version: find_package(COLMAP 3.4 REQUIRED)
find_package(COLMAP REQUIRED)
if(COLMAP_FOUND)
  message(STATUS "Found COLMAP INCLUDE DIR ${COLMAP_INCLUDE_DIRS}")
  message(STATUS "Found COLMAP LINK DIR ${COLMAP_LINK_DIRS}")
  message(STATUS "Found COLMAP LIB DIR ${COLMAP_LIBRARIES}")
  message(STATUS "COLMAP ENABLED")
  add_definitions(-DCOLMAP_ENABLED)
else(COLMAP_FOUND)
  message(FATAL "COLMAP not found!")
endif(COLMAP_FOUND)

find_package(Gopt REQUIRED)
if(GOPT_FOUND)
  message(STATUS "Found GOPT_INCLUDE_DIRs ${GOPT_INCLUDE_DIRS}")
  message(STATUS "Found GOPT_LINK_DIRs ${GOPT_LINK_DIRS}")
  message(STATUS "Found GOPT_LIBRARIES ${GOPT_LIBRARIES}")
else(GOPT_FOUND)
  message(FATAL "GOPT not found!")
endif(GOPT_FOUND)

include_directories(${COLMAP_INCLUDE_DIRS} ${GOPT_INCLUDE_DIRS})
link_directories(${COLMAP_LINK_DIRS} ${GOPT_LINK_DIRS})


configure_file(run_global_sfm.sh.in ${PROJECT_SOURCE_DIR}/run_global_sfm.sh)
# install(FILES "${PROJECT_SOURCE_DIR}/run_global_sfm.sh"
#         DESTINATION "../../scripts")

add_library(global_sfm
    global_mapper_controller.h global_mapper_controller.cc
    global_mapper.h global_mapper.cc
    utils.h utils.cc
)
target_link_libraries(global_sfm ${COLMAP_LIBRARIES} ${GOPT_LIBRARIES})

add_executable(run_global_mapper run_global_mapper.cc)
target_link_libraries(run_global_mapper
    global_sfm
    ${COLMAP_LIBRARIES}
    ${GOPT_LIBRARIES}
)
