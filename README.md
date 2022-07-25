Graph Optimizer
===============

[![Build Status](https://travis-ci.com/AIBluefisher/GraphOptim.svg?branch=main)](https://travis-ci.com/AIBluefisher/GraphOptim) [![License](https://img.shields.io/badge/license-BSD--3--Clause-blue)](./LICENSE)


This repo contains the official implementation of our CVPR 2021 paper - [Hybrid Rotation Averaging: A Fast and Robust Rotation Averaging Approach](https://arxiv.org/pdf/2101.09116.pdf). This library contains not only rotation averaging solvers, but also some popular methods in 3D vision, such as translation averaging, clustering, etc. The library is designed to deal with large scale optimization problems, and is easy to extend. Feel free to contribute to this project.

## 1. Features

* A library which solves the optimization problems in 3D vision.
* A template-based graph module which is easy to extend and to manipulate graph structures.
* Rotation averaging solvers achieves state-of-the-art.
* Translation averaging solvers (LUD, LiGT, BATA).
* Clustering methods (coming soon).
* Bundle adjustment (needs more time to prepare).

## 2. Compilation

The library is compiled and tested on Ubuntu 16.04. We would like to support more platforms in the future.

### 2.1 Basic Requirements

This project requires Eigen 3.2. And Ceres 1.14.0 is currently used for nonlinear optimization. You can install all the dependencies through the `./scripts/dependencies.sh`.

```sh
bash ./scripts/dependencies.sh
```

### 2.3 Build GraphOptim

```sh
cd GraphOptim
mkdir build && cd build
cmake ..
make -j8
sudo make install
```

### 2.4 Build GlobalSfM

Once we installed GraphOptim, we can use it as an external library. And also, we can try the provided global SfM application. 

```sh
cd applications
mkdir build && cd build
cmake ..
make -j8
```

## 3. Running Examples

### 3.1 Rotation Averaging

```sh
./build/bin/rotation_estimator --g2o_filename=./data/synthetic/20_2.g2o
```

You can also try other `g2o` files.

### 3.2 Translation Averaging

The translation averaging methods are decoupled from another project.

```sh
./build/bin/position_estimator --g2o_filename=./data/synthetic/20_2.g2o
```

### 3.3 Global Structure from Motion

```sh
./applications/run_global_sfm.sh $DATASET_PATH $OUTPUT_PATH $VOC_TREE_PATH $MOST_SIMILAR_IMAGES_NUM
```
You may also want to try different rotation averaging and translation averaging solvers by setting (just appending the parameters at the end of your command):
- `ROTATION_ESTIMATOR_TYPE`: [ROBUST_L1L2, HYBRID]
- `POSITION_ESTIMATOR_TYPE`: [LUD, LIGT]


### Citation

If you find the code useful for your research, please use the following `BibTeX` entry:
```
@inproceedings{DBLP:conf/cvpr/Chen0K21,
  author    = {Yu Chen and
               Ji Zhao and
               Laurent Kneip},
  title     = {Hybrid Rotation Averaging: {A} Fast and Robust Rotation Averaging
               Approach},
  booktitle = {{IEEE} Conference on Computer Vision and Pattern Recognition},
  pages     = {10358--10367},
  year      = {2021}
}
```