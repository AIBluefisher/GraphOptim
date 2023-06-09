#ifndef GEOMETRY_ALIGN_POINT_CLOUDS_H_
#define GEOMETRY_ALIGN_POINT_CLOUDS_H_

#include <vector>
#include <Eigen/Core>

namespace gopt {

// Computes the orientation, position, and scale factor for the transformation
// between two corresponding 3D point sets A and B such as they are related by:
//
//     B = s * R * A + t
//
// where A is "left" and B is "right". Implementation is based on the paper by
// Umeyama "Least-squares estimation of transformation parameters between two
// point patterns".
void AlignPointCloudsUmeyama(const std::vector<Eigen::Vector3d>& left,
                             const std::vector<Eigen::Vector3d>& right,
                             Eigen::Matrix3d* rotation,
                             Eigen::Vector3d* translation, double* scale);

// Adds weights for each match
// The previous objective function E = Sum(|| yi - (c.R.xi + T) ||^2) becomes
// Sum(wi * || yi - (c.R.xi + T) ||^2)
// The weights must be positive
void AlignPointCloudsUmeyamaWithWeights(
    const std::vector<Eigen::Vector3d>& left,
    const std::vector<Eigen::Vector3d>& right,
    const std::vector<double>& weights, Eigen::Matrix3d* rotation,
    Eigen::Vector3d* translation, double* scale);

}  // namespace gopt

#endif  // GEOMETRY_ALIGN_POINT_CLOUDS_H_
