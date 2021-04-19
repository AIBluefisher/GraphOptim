#!/bin/bash

sudo apt-get update

sudo apt-get install git cmake build-essential \
  libgoogle-glog-dev libgflags-dev libgtest-dev

git clone https://github.com/eigenteam/eigen-git-mirror.git
cd eigen-git-mirror
git checkout 3.2.10
mkdir build && cd build
cmake .. && make
sudo make install

sudo apt-get install libatlas-base-dev libsuitesparse-dev
git clone https://ceres-solver.googlesource.com/ceres-solver
cd ceres-solver
git checkout $(git describe --tags) # Checkout the latest release
mkdir build && cd build
cmake .. -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF
make -j8
sudo make install