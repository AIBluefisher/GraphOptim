#!/bin/bash

sudo apt-get update

sudo apt-get install git cmake build-essential \
  libgoogle-glog-dev libgflags-dev libgtest-dev libeigen3-dev

mkdir tmp && cd tmp

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
