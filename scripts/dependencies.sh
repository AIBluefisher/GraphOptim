#!/bin/bash

sudo apt-get update

sudo apt-get install git cmake build-essential \
  libgoogle-glog-dev libgflags-dev libgtest-dev libeigen3-dev

mkdir tmp && cd tmp

# Install Ceres-Solver
sudo apt-get install libatlas-base-dev libsuitesparse-dev
git clone https://ceres-solver.googlesource.com/ceres-solver
cd ceres-solver
# Latest release of ceres not provides FindEigen3.cmake
git checkout 1.14.0
mkdir build && cd build
cmake .. -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF
make -j8
sudo make install
cd ../../

# Install COLMAP
sudo apt-get install \
    git \
    cmake \
    build-essential \
    libboost-program-options-dev \
    libboost-filesystem-dev \
    libboost-graph-dev \
    libboost-system-dev \
    libboost-test-dev \
    libsuitesparse-dev \
    libfreeimage-dev \
    libmetis-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libglew-dev \
    qtbase5-dev \
    libqt5opengl5-dev \
    libcgal-dev

sudo apt-get install libcgal-qt5-dev

git clone https://github.com/colmap/colmap.git
cd colmap
git checkout dev
mkdir build
cd build
cmake ..
make -j
sudo make install
cd ../../

cd ..
rm -rf tmp

if [ -d "/usr/src/gtest" ]; then
  cd /usr/src/gtest
  sudo mkdir build && cd build
  sudo cmake .. && sudo make
  sudo cp libgtest* /usr/lib/
  cd ../ && sudo rm -rf build
else
  echo "Could not find gtest"
fi
