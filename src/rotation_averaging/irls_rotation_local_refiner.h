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

#ifndef ROTATION_AVERAGING_IRLS_ROTATION_LOCAL_REFINE_H_
#define ROTATION_AVERAGING_IRLS_ROTATION_LOCAL_REFINE_H_

#include <vector>
#include <utility>
#include <unordered_map>

#include "geometry/rotation_utils.h"
#include "util/types.h"

#include <Eigen/Core>
#include <Eigen/SparseCore>

namespace gopt {

class IRLSRotationLocalRefiner {
 public:
  struct IRLSRefinerOptions {
    bool verbose = true;
    
    int num_threads = 8;

    // The number of iterative reweighted least squares iterations to perform.
    int max_num_irls_iterations = 10;

    // Average step size threshold to termininate the IRLS minimization
    double irls_step_convergence_threshold = 0.001;

    // This is the point where the Huber-like cost function switches from L1 to
    // L2.
    double irls_loss_parameter_sigma = geometry::DegToRad(5.0);
  };

  IRLSRotationLocalRefiner(
      const int num_orientations, const int num_edges,
      const IRLSRefinerOptions& options);

  void SetInitTangentSpaceStep(const Eigen::VectorXd& tangent_space_step);

  void SetViewIdToIndex(const std::unordered_map<image_t, int>& view_id_to_index);

  void SetSparseMatrix(const Eigen::SparseMatrix<double>& sparse_matrix);

  bool SolveIRLS(
      const std::unordered_map<ImagePair, TwoViewGeometry>& relative_rotations,
      std::unordered_map<image_t, Eigen::Vector3d>* global_rotations);

  // We keep one of the rotations as constant to remove the ambiguity of the
  // linear system.
  static const int kConstantRotationIndex = -1;

 private:
  // Update the global orientations using the current value in the
  // rotation_change.
  void UpdateGlobalRotations(
      std::unordered_map<image_t, Eigen::Vector3d>* global_rotations);

  // Computes the relative rotation error based on the current global
  // orientation estimates.
  void ComputeResiduals(
    const std::unordered_map<ImagePair, TwoViewGeometry>& relative_rotations,
    std::unordered_map<image_t, Eigen::Vector3d>* global_rotations);

  // Computes the average size of the most recent step of the algorithm.
  // The is the average over all non-fixed global_rotations_ of their
  // rotation magnitudes.
  double ComputeAverageStepSize();

  const IRLSRefinerOptions options_;

  // Map of image_ts to the corresponding positions of the view's orientation in
  // the linear system.
  std::unordered_map<image_t, int> view_id_to_index_;

  // The sparse matrix used to maintain the linear system. This is matrix A in
  // Ax = b.
  Eigen::SparseMatrix<double> sparse_matrix_;

  // x in the linear system Ax = b.
  Eigen::VectorXd tangent_space_step_;

  // b in the linear system Ax = b.
  Eigen::VectorXd tangent_space_residual_;

};

}  // namespace gopt

#endif  // ROTATION_AVERAGING_IRLS_ROTATION_LOCAL_REFINE_H_
