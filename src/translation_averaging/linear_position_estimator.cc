#include "translation_averaging/linear_position_estimator.h"

#include <unordered_set>
#include <omp.h>

#include "geometry/rotation.h"
#include "geometry/rotation_utils.h"
#include "utils/progressbar.h"

#include "Spectra/SymEigsShiftSolver.h"

namespace gopt {

LinearPositionEstimator::LinearPositionEstimator(
      const std::vector<TrackElements>& tracks,
      const std::unordered_map<image_t, KeypointsMat>& normalized_keypoints)
  : tracks_(tracks),
    normalized_keypoints_(normalized_keypoints) {}

bool LinearPositionEstimator::EstimatePositions(
    const std::unordered_map<ImagePair, TwoViewGeometry>& view_pairs,
    const std::unordered_map<image_t, Eigen::Vector3d>& orientation,
    std::unordered_map<image_t, Eigen::Vector3d>* positions) {
  InitializeIndexMapping(orientation);

  const size_t num_images = orientation.size();
  Eigen::MatrixXd A_lr = Eigen::MatrixXd::Zero(tracks_.size(), 3 * num_images);
  Eigen::MatrixXd LtL = SetUpLinearSystem(orientation, &A_lr);

  Spectra::DenseSymShiftSolve<double> op(LtL);
  Spectra::SymEigsShiftSolver<Spectra::DenseSymShiftSolve<double>>
      eigs(op, 1, 8, 0.0);
  eigs.init();
  eigs.compute(Spectra::SortRule::LargestMagn);

  if (eigs.info() != Spectra::CompInfo::Successful) {
    LOG(ERROR) << "Solve for SVD Failed!";
    return false;
  }

  const Eigen::VectorXd eigen_values = eigs.eigenvalues();
  LOG(INFO) << "Eigenvalues: " << eigen_values.transpose();

  Eigen::VectorXd eigen_vectors = Eigen::VectorXd::Zero(3 * num_images);
  eigen_vectors.bottomRows(3 * (num_images - 1)) = eigs.eigenvectors();

  IdentifySign(A_lr, &eigen_vectors);

  // Set the estimated positions.
  for (const auto& image_id_index : image_id_to_index_) {
    const int index = image_id_index.second;
    const image_t image_id = image_id_index.first;
    (*positions)[image_id] = eigen_vectors.segment<3>(index + 3);
  }

  return true;
}

void LinearPositionEstimator::InitializeIndexMapping(
    const std::unordered_map<image_t, Eigen::Vector3d>& orientations) {
  // Create a mapping from the view id to the index of the linear system.
  int index = kConstantImageIndex;
  image_id_to_index_.reserve(orientations.size());
  for (const auto& iter : orientations) {
    const image_t image_id = iter.first;
    image_id_to_index_[image_id] = index;
    index += 3;
  }
}

Eigen::MatrixXd LinearPositionEstimator::SetUpLinearSystem(
    const std::unordered_map<image_t, Eigen::Vector3d>& orientation,
    Eigen::MatrixXd* A_lr) {
  const size_t num_images = orientation.size();
  LOG(INFO) << "Num images: " << num_images;
  Eigen::MatrixXd LtL =
      Eigen::MatrixXd::Zero(3 * (num_images - 1), 3 * (num_images - 1));
  LOG(INFO) << "Num tracks: " << tracks_.size();
  ProgressBar progress_bar(tracks_.size());
  progress_bar.SetTodoChar(" ");
  progress_bar.SetDoneChar("█");

  // #pragma omp parallel for shared(A_lr, LtL)
  for (size_t i = 0; i < tracks_.size(); i++) {
    const TrackElements& track_elements = tracks_[i];
    TrackElement left_track_element, right_track_element;
    SelectLeftRightBaseViews(
        orientation, track_elements, &left_track_element, &right_track_element);
    const image_t left_base_image_id = left_track_element.image_id;
    const image_t right_base_image_id = right_track_element.image_id;
    const point2D_t left_base_point_id = left_track_element.point2D_idx;
    const point2D_t right_base_point_id = right_track_element.point2D_idx;

    const int left_base_image_index = image_id_to_index_.at(left_base_image_id);
    const int right_base_image_index = image_id_to_index_.at(right_base_image_id);

    const Eigen::Matrix3d Rl = AngleAxisToRotationMatrix(
        orientation.at(left_base_image_id));
    const Eigen::Matrix3d Rr = AngleAxisToRotationMatrix(
        orientation.at(right_base_image_id));
    
    const Eigen::Matrix3d Rlr = Rr * Rl.transpose();
    const Eigen::Vector3d Xl =
        normalized_keypoints_.at(left_base_image_id).row(left_base_point_id).transpose();
    const Eigen::Vector3d Xr =
        normalized_keypoints_.at(right_base_image_id).row(right_base_point_id).transpose();

    const double theta_lr = Theta12(Rlr, Xl, Xr);
    const Eigen::Matrix<double, 1, 3> a_lr = A12(Rlr, Xl, Xr);

    (*A_lr).row(i).block<1, 3>(0, left_base_image_index + 3) = a_lr * Rr;
    (*A_lr).row(i).block<1, 3>(0, right_base_image_index + 3) = -a_lr * Rr;

    Eigen::MatrixXd local_coefficient_mat = Eigen::MatrixXd::Zero(3, 3 * num_images);
    for (size_t k = 0; k < track_elements.size(); k++) {
      const TrackElement& element = track_elements[k];
      if (element.image_id == left_track_element.image_id) {
        continue;
      }

      const int image_index = image_id_to_index_.at(element.image_id);
      const Eigen::Matrix3d Rk =
          AngleAxisToRotationMatrix(orientation.at(element.image_id));
      const Eigen::Vector3d Xk =
          normalized_keypoints_.at(element.image_id).row(element.point2D_idx).transpose();
      const Eigen::Matrix3d Rlk = Rk * Rl.transpose();
      const Eigen::Matrix3d Xk_skew_mat = geometry::CrossProductMatrix(Xk);
      
      const Eigen::Matrix3d A = Xk_skew_mat * Rlk * Xl * a_lr * Rr;
      const Eigen::Matrix3d B = theta_lr * theta_lr * Xk_skew_mat * Rk;
      const Eigen::Matrix3d C = -(A + B);

      local_coefficient_mat.setZero();
      local_coefficient_mat.block<3, 3>(0, left_base_image_index + 3) += C;
      local_coefficient_mat.block<3, 3>(0, image_index + 3) += B;
      local_coefficient_mat.block<3, 3>(0, right_base_image_index + 3) += A;

      Eigen::MatrixXd Ltl_l = C.transpose() * local_coefficient_mat;
      Eigen::MatrixXd Ltl_k = B.transpose() * local_coefficient_mat;
      Eigen::MatrixXd Ltl_r = A.transpose() * local_coefficient_mat;
      
      // #pragma omp critical
      if (left_base_image_index > 0) {
        LtL.middleRows<3>(left_base_image_index) += Ltl_l.rightCols(Ltl_l.cols() - 3);
      }

      if (image_index > 0) {
        LtL.middleRows<3>(image_index) += Ltl_k.rightCols(Ltl_k.cols() - 3);
      }

      if (right_base_image_index > 0) {
        LtL.middleRows<3>(right_base_image_index) += Ltl_r.rightCols(Ltl_r.cols() - 3);
      }
    }
    progress_bar.Update();
  }
  return LtL;
}

void LinearPositionEstimator::SelectLeftRightBaseViews(
    const std::unordered_map<image_t, Eigen::Vector3d>& orientation,
    const TrackElements& track_elements,
    TrackElement* track_element1,
    TrackElement* track_element2) {
  double max_theta = 0;
  for (size_t i = 0; i < track_elements.size(); i++) {
    const TrackElement& element1 = track_elements[i];
    const image_t image_id1 = element1.image_id;
    const Eigen::Vector3d X1 = normalized_keypoints_.at(image_id1).row(
        element1.point2D_idx).transpose();
    const Eigen::Matrix3d R1 = AngleAxisToRotationMatrix(
        orientation.at(image_id1));

    for (size_t j = i + 1; j < track_elements.size(); j++) {
      const TrackElement& element2 = track_elements[j];
      const image_t image_id2 = element2.image_id;
      const Eigen::Vector3d X2 = normalized_keypoints_.at(image_id2).row(
          element2.point2D_idx).transpose();
      const Eigen::Matrix3d R2 = AngleAxisToRotationMatrix(
          orientation.at(image_id2));
      const Eigen::Matrix3d R12 = R2 * R1.transpose();
      const double theta12 = Theta12(R12, X1, X2);
      if (max_theta < theta12) {
        max_theta = theta12;
        *track_element1 = element1;
        *track_element2 = element2;
      }
    }
  }
}

double LinearPositionEstimator::Theta12(const Eigen::Matrix3d& R12,
                                        const Eigen::Vector3d& X1,
                                        const Eigen::Vector3d& X2) {
  const Eigen::Vector3d theta12 = geometry::CrossProductMatrix(X2) * R12 * X1;
  return theta12.norm();
}

const Eigen::Matrix<double, 1, 3> LinearPositionEstimator::A12(
    const Eigen::Matrix3d& R12,
    const Eigen::Vector3d& X1,
    const Eigen::Vector3d& X2) {
  const Eigen::Matrix<double, 1, 3> a12 = (
    geometry::CrossProductMatrix(R12 * X1) * X2
  ).transpose() * geometry::CrossProductMatrix(X2);
  return a12;
}

void LinearPositionEstimator::IdentifySign(const Eigen::MatrixXd& A_lr,
                                           Eigen::VectorXd* eigen_vectors) {
  const Eigen::VectorXd judge_value = A_lr * (*eigen_vectors);
  const int positive_count = (judge_value.array() > 0.0).cast<int>().sum();
  const int negative_count = judge_value.rows() - positive_count;
  LOG(INFO) << "positive count: " << positive_count;
  LOG(INFO) << "negative count: " << negative_count;
  if (positive_count < negative_count) {
    *eigen_vectors *= -1;
  }
}

}  // namespace gopt
