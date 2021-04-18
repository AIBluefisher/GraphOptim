#include "translation_averaging/lud_position_estimator.h"

#include <unordered_set>

#include <ceres/rotation.h>
#include <Eigen/SparseCore>
#include <glog/logging.h>

#include "solver/constrained_l1_solver.h"
#include "util/map_util.h"

namespace gopt {
namespace {

Eigen::Vector3d GetRotatedTranslation(
    const Eigen::Vector3d& rotation_angle_axis,
    const Eigen::Vector3d& translation) {
  Eigen::Matrix3d rotation;
  ceres::AngleAxisToRotationMatrix(
      rotation_angle_axis.data(),
      ceres::ColumnMajorAdapter3x3(rotation.data()));
  return rotation.transpose() * translation;
}

}  // namespace

LUDPositionEstimator::LUDPositionEstimator(
    const LUDPositionEstimator::Options& options)
    : options_(options) {
  CHECK_GT(options_.max_num_iterations, 0);
  CHECK_GT(options_.max_num_reweighted_iterations, 0);
}

bool LUDPositionEstimator::EstimatePositions(
    const std::unordered_map<ImagePair, TwoViewGeometry>& view_pairs,
    const std::unordered_map<image_t, Eigen::Vector3d>& orientations,
    std::unordered_map<image_t, Eigen::Vector3d>* positions) {
  CHECK_NOTNULL(positions)->clear();

  InitializeIndexMapping(view_pairs, orientations);
  const size_t num_views = view_id_to_index_.size();
  const size_t num_view_pairs = view_id_pair_to_index_.size();

  // Set up the linear system.
  SetupConstraintMatrix(view_pairs, orientations);
  Eigen::VectorXd solution;
  solution.setZero(constraint_matrix_.cols());

  // Create the lower bound constraint enforcing that all scales are > 1.
  Eigen::SparseMatrix<double> geq_mat(num_view_pairs,
                                      constraint_matrix_.cols());
  for (size_t i = 0; i < num_view_pairs; i++) {
    geq_mat.insert(i, 3 * (num_views - 1) + i) = 1.0;
  }
  Eigen::VectorXd geq_vec(num_view_pairs);
  geq_vec.setConstant(1.0);

  Eigen::VectorXd b(constraint_matrix_.rows());
  b.setZero();

  // Solve for camera positions by solving a constrained L1 problem to enforce
  // all relative translations scales > 1.
  ConstrainedL1Solver::Options l1_options;
  ConstrainedL1Solver solver(
      l1_options, constraint_matrix_, b, geq_mat, geq_vec);
  solver.Solve(&solution);

  // Set the estimated positions.
  for (const auto& view_id_index : view_id_to_index_) {
    const int index = view_id_index.second;
    const image_t view_id = view_id_index.first;
    if (index == kConstantViewIndex) {
      (*positions)[view_id] = Eigen::Vector3d::Zero();
    } else {
      (*positions)[view_id] = solution.segment<3>(index);
    }
  }

  return true;
}

void LUDPositionEstimator::InitializeIndexMapping(
    const std::unordered_map<ImagePair, TwoViewGeometry>& view_pairs,
    const std::unordered_map<image_t, Eigen::Vector3d>& orientations) {
  std::unordered_set<image_t> views;
  for (const auto& view_pair : view_pairs) {
    if (ContainsKey(orientations, view_pair.first.first) &&
        ContainsKey(orientations, view_pair.first.second)) {
      views.insert(view_pair.first.first);
      views.insert(view_pair.first.second);
    }
  }

  // Create a mapping from the view id to the index of the linear system.
  int index = kConstantViewIndex;
  view_id_to_index_.reserve(views.size());
  for (const image_t view_id : views) {
    view_id_to_index_[view_id] = index;
    index += 3;
  }

  // Create a mapping from the view id pair to the index of the linear system.
  view_id_pair_to_index_.reserve(view_pairs.size());
  for (const auto& view_pair : view_pairs) {
    if (ContainsKey(view_id_to_index_, view_pair.first.first) &&
        ContainsKey(view_id_to_index_, view_pair.first.second)) {
      view_id_pair_to_index_[view_pair.first] = index;
      ++index;
    }
  }
}

void LUDPositionEstimator::SetupConstraintMatrix(
    const std::unordered_map<ImagePair, TwoViewGeometry>& view_pairs,
    const std::unordered_map<image_t, Eigen::Vector3d>& orientations) {
  constraint_matrix_.resize(
      3 * view_id_pair_to_index_.size(),
      3 * (view_id_to_index_.size() - 1) + view_pairs.size());

  // Add the camera to camera constraints.
  std::vector<Eigen::Triplet<double>> triplet_list;
  triplet_list.reserve(9 * view_pairs.size());
  int row = 0;
  for (const auto& view_pair : view_pairs) {
    const ImagePair view_id_pair = view_pair.first;
    if (!ContainsKey(view_id_to_index_, view_id_pair.first) ||
        !ContainsKey(view_id_to_index_, view_id_pair.second)) {
      continue;
    }

    const int view1_index = FindOrDie(view_id_to_index_, view_id_pair.first);
    const int view2_index = FindOrDie(view_id_to_index_, view_id_pair.second);
    const int scale_index =
        FindOrDieNoPrint(view_id_pair_to_index_, view_id_pair);

    // Rotate the relative translation so that it is aligned to the global
    // orientation frame.
    const Eigen::Vector3d translation_direction =
        GetRotatedTranslation(FindOrDie(orientations, view_id_pair.first),
                              view_pair.second.translation_2);

    // Add the constraint for view 1 in the minimization:
    //   position2 - position1 - scale_1_2 * translation_direction.
    if (view1_index != kConstantViewIndex) {
      triplet_list.emplace_back(row + 0, view1_index + 0, -1.0);
      triplet_list.emplace_back(row + 1, view1_index + 1, -1.0);
      triplet_list.emplace_back(row + 2, view1_index + 2, -1.0);
    }

    // Add the constraint for view 2 in the minimization:
    //   position2 - position1 - scale_1_2 * translation_direction.
    if (view2_index != kConstantViewIndex) {
      triplet_list.emplace_back(row + 0, view2_index + 0, 1.0);
      triplet_list.emplace_back(row + 1, view2_index + 1, 1.0);
      triplet_list.emplace_back(row + 2, view2_index + 2, 1.0);
    }

    // Add the constraint for scale in the minimization:
    //   position2 - position1 - scale_1_2 * translation_direction.
    triplet_list.emplace_back(row + 0, scale_index, -translation_direction[0]);
    triplet_list.emplace_back(row + 1, scale_index, -translation_direction[1]);
    triplet_list.emplace_back(row + 2, scale_index, -translation_direction[2]);

    row += 3;
  }

  constraint_matrix_.setFromTriplets(triplet_list.begin(), triplet_list.end());

  VLOG(2) << view_pairs.size() << " camera to camera constraints were added "
                                  "to the position estimation problem.";
}

}  // namespace gopt
