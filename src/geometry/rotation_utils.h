#ifndef GEOMETRY_ROTATION_UTILS_H_
#define GEOMETRY_ROTATION_UTILS_H_

#include <algorithm>
#include <cmath>
#include <vector>
#include <unordered_map>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include "util/random.h"
#include "util/types.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif

namespace gopt {
namespace geometry {
static RandomNumberGenerator rng(56);
static const double kRadToDeg = 180.0 / M_PI;
static const double kDegToRad = M_PI / 180.0;

double RadToDeg(double angle_radians);

double DegToRad(double angle_degrees);

double Clamp(const double val, const double min, const double max);

Eigen::MatrixXd ProjectToSOd(const Eigen::MatrixXd &M);

// Rotates the "rotation" set of orientations such that the orientations are
// most closely aligned in an L2 sense. That is, "rotation" is transformed such
// that R_rotation * R_gt_rotation^t is minimized.
void AlignRotations(const std::vector<Eigen::Vector3d>& gt_rotation,
                    std::vector<Eigen::Vector3d>* rotation);

// Aligns rotations to the ground truth rotations via a similarity
// transformation.
void AlignOrientations(
    const std::unordered_map<image_t, Eigen::Vector3d>& gt_rotations,
    std::unordered_map<image_t, Eigen::Vector3d>* rotations);

// Use Ceres to perform a stable composition of rotations. This is not as
// efficient as directly composing angle axis vectors (see the old
// implementation commented above) but is more stable.
Eigen::Vector3d MultiplyRotations(const Eigen::Vector3d& rotation1,
                                  const Eigen::Vector3d& rotation2);

// Computes R_ij = R_j * R_i^t.
Eigen::Vector3d RelativeRotationFromTwoRotations(
    const Eigen::Vector3d& rotation1, const Eigen::Vector3d& rotation2,
    const double noise);

// Computes R_ij = R_j * R_i^t.
Eigen::Vector3d RelativeRotationFromTwoRotations(
    const Eigen::Vector3d& rotation1, const Eigen::Vector3d& rotation2);

// return R_j = R_ij * R_i.
Eigen::Vector3d ApplyRelativeRotation(
    const Eigen::Vector3d& rotation1,
    const Eigen::Vector3d& relative_rotation);

Eigen::Vector3d RelativeTranslationFromTwoPositions(
    const Eigen::Vector3d& position1, const Eigen::Vector3d& position2,
    const Eigen::Vector3d& rotation1);

}  // namespace geometry
}  // namespace gopt

#endif  // GEOMETRY_ROTATION_UTILS_H_
