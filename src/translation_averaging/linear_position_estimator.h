#ifndef TRANSLATION_AVERAGING_H_
#define TRANSLATION_AVERAGING_H_

#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include "geometry/track_builder.h"
#include "translation_averaging/position_estimator.h"

namespace gopt {

class LinearPositionEstimator : public PositionEstimator {
 public:
  LinearPositionEstimator(
      const std::vector<TrackElements>& tracks,
      const std::unordered_map<image_t, KeypointsMat>& normalized_keypoints);
  
  bool EstimatePositions(
      const std::unordered_map<ImagePair, TwoViewGeometry>& view_pairs,
      const std::unordered_map<image_t, Eigen::Vector3d>& orientation,
      std::unordered_map<image_t, Eigen::Vector3d>* positions) override;

 private:
  void InitializeIndexMapping(
      const std::unordered_map<image_t, Eigen::Vector3d>& orientations);

  Eigen::MatrixXd SetUpLinearSystem(
      const std::unordered_map<image_t, Eigen::Vector3d>& orientation,
      Eigen::MatrixXd* A_lr);

  void SelectLeftRightBaseViews(
      const std::unordered_map<image_t, Eigen::Vector3d>& orientation,
      const TrackElements& track_elements,
      TrackElement* track_element1,
      TrackElement* track_element2);

  double Theta12(const Eigen::Matrix3d& R12,
                 const Eigen::Vector3d& X1,
                 const Eigen::Vector3d& X2);

  const Eigen::Matrix<double, 1, 3> A12(const Eigen::Matrix3d& R12,
                                        const Eigen::Vector3d& X1,
                                        const Eigen::Vector3d& X2);

  void IdentifySign(const Eigen::MatrixXd& A_lr,
                    Eigen::VectorXd* eigen_vectors);

  static const int kConstantImageIndex = -3;
  std::unordered_map<image_t, int> image_id_to_index_;

  const std::vector<TrackElements> tracks_;
  const std::unordered_map<image_t, KeypointsMat> normalized_keypoints_;
};

}  // namespace gopt

#endif  // TRANSLATION_AVERAGING_H_
