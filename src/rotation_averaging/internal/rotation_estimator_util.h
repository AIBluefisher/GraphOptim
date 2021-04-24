// BSD 3-Clause License

// Copyright (c) 2021, Chenyu
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.

// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef ROTATION_AVERAGING_INTERNAL_ROTATION_ESTIMATOR_UTIL_H_
#define ROTATION_AVERAGING_INTERNAL_ROTATION_ESTIMATOR_UTIL_H_

#include <unordered_map>

#include <Eigen/Core>
#include <Eigen/SparseCore>

#include "util/hash.h"
#include "util/types.h"
#include "util/map_util.h"

namespace gopt {
namespace internal {

// The id of rotations is re-indexed for convenience of matrix manipulation.
static inline void ViewIdToAscentIndex(
    const std::unordered_map<image_t, Eigen::Vector3d>& rotations,
    std::unordered_map<image_t, int>* view_id_to_index) {
  std::vector<image_t> vec_ids;
  for (auto rotation : rotations) {
    vec_ids.push_back(rotation.first);
  }

  std::sort(vec_ids.begin(), vec_ids.end());

  for (size_t i = 0; i < vec_ids.size(); i++) {
    (*view_id_to_index)[vec_ids[i]] = i;
  }
}

// Sets up the sparse linear system such that dR_ij = dR_j - dR_i. This is the
// first-order approximation of the angle-axis rotations. This should only be
// called once.
static inline void SetupLinearSystem(
    const std::unordered_map<ImagePair, TwoViewGeometry>& relative_rotations,
    const size_t num_rotations,
    const std::unordered_map<image_t, int>& view_id_to_index,
    Eigen::SparseMatrix<double>* sparse_matrix) {
  // The rotation change is one less than the number of global rotations because
  // we keep one rotation constant.
  (*sparse_matrix).resize(
      relative_rotations.size() * 3, (num_rotations - 1) * 3);

  const int kStartRotationIndex = -1;

  // For each relative rotation constraint, add an entry to the sparse
  // matrix. We use the first order approximation of angle axis such that:
  // R_ij = R_j - R_i. This makes the sparse matrix just a bunch of identity
  // matrices.
  int rotation_error_index = 0;
  std::vector<Eigen::Triplet<double>> triplet_list;
  for (const auto& relative_rotation : relative_rotations) {
    const int view1_index =
        FindOrDie(view_id_to_index, relative_rotation.first.first) - 1;
    if (view1_index != kStartRotationIndex) {
      triplet_list.emplace_back(3 * rotation_error_index + 0,
                                3 * view1_index + 0, -1.0);
      triplet_list.emplace_back(3 * rotation_error_index + 1,
                                3 * view1_index + 1, -1.0);
      triplet_list.emplace_back(3 * rotation_error_index + 2,
                                3 * view1_index + 2, -1.0);
    }

    const int view2_index =
        FindOrDie(view_id_to_index, relative_rotation.first.second) - 1;
    if (view2_index != kStartRotationIndex) {
      triplet_list.emplace_back(3 * rotation_error_index + 0,
                                3 * view2_index + 0, 1.0);
      triplet_list.emplace_back(3 * rotation_error_index + 1,
                                3 * view2_index + 1, 1.0);
      triplet_list.emplace_back(3 * rotation_error_index + 2,
                                3 * view2_index + 2, 1.0);
    }

    ++rotation_error_index;
  }

  sparse_matrix->setFromTriplets(triplet_list.begin(), triplet_list.end());
}

}  // namespace internal
}  // namespace gopt

#endif  // ROTATION_AVERAGING_INTERNAL_ROTATION_ESTIMATOR_UTIL_H_
