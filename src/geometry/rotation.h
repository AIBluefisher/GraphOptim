#ifndef SRC_GEOMETRY_ROTATION_H_
#define SRC_GEOMETRY_ROTATION_H_

#include <Eigen/Core>

namespace gopt {

Eigen::Vector4d NormalizeQuaternion(const Eigen::Vector4d& quat);

Eigen::Vector3d RotationMatrixToAngleAxis(const Eigen::Matrix3d& R);
Eigen::Vector4d RotationMatrixToQuaternion(const Eigen::Matrix3d& R);

Eigen::Matrix3d AngleAxisToRotationMatrix(const Eigen::Vector3d& angle_axis);
Eigen::Matrix3d QuaternionToRotationMatrix(const Eigen::Vector4d& quaternion);

Eigen::Vector3d QuaternionToAngleAxis(const Eigen::Vector4d& quat);
Eigen::Vector4d AngleAxisToQuaternion(const Eigen::Vector3d& angle_axis);

}  // namespace gopt

#endif  // SRC_GEOMETRY_ROTATION_H_
