# Graph Optimizer

This repo contains the official implementation of our CVPR 2021 paper - [Hybrid Rotation Averaging: A Fast and Robust Rotation Averaging Approach](https://arxiv.org/pdf/2101.09116.pdf). This library contains not only rotation averaging solvers, but also some popular methods in 3D vision, such as translation averaging, clustering, etc. The library is designed for dealing with large scale optimization problems, and is easy to extend. Feel free to contribute to this project.

## Features

* A library which solves the optimization problems in 3D vision.
* A template-based graph module which is easy to extend and to manipulate graph structures.
* Rotation averaging solvers achieves state-of-the-art.
* Translation averaging solvers (LUD, BATA).
* Clustering methods (coming soon).
* Bundle adjustment (needs more time to prepare).

## 2. Compilation

### 2.1 Basic Requirements

```sh
sudo apt-get install git cmake build-essential \
  libgoogle-glog-dev libgflags-dev libgtest-dev libeigen3-dev
```

### 2.2 Ceres Solver

Ceres currently is used for stable conversions between different rotation representations.
I'm managing to remove this dependency.

```sh
sudo apt-get install libatlas-base-dev libsuitesparse-dev
git clone https://ceres-solver.googlesource.com/ceres-solver
cd ceres-solver
git checkout $(git describe --tags) # Checkout the latest release
mkdir build && cd build
cmake .. -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF
make -j8
sudo make install
```

### 2.3 Build `gopt`

```sh
cd GraphOptim
mkdir build && cd build
cmake ..
make -j8
```

***NOTE: If the GTest has been installed but cannot be found by cmake, try the patch below:***
```sh
cd /usr/src/gtest
sudo mkdir build && cd build
sudo cmake .. && make
sudo cp libgtest* /usr/lib/
cd ../ && sudo rm -rf build
```

## 3. Running Examples

### 3.1 Rotation Averaging

```sh
./build/bin/rotation_estimator --g2o_filename=../../data/synthetic/20_2.g2o
```

You can also try other `g2o` files.

### 3.2 Translation Averaging

The translation averaging methods are decoupled from another projects, and are not fully tested.

```sh
./build/bin/position_estimator --g2o_filename=../../data/synthetic/20_2.g2o
```
