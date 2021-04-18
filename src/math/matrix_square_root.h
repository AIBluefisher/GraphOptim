#ifndef MATH_MATRIX_SQUARE_ROOT_H
#define MATH_MATRIX_SQUARE_ROOT_H

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

namespace gopt {

Eigen::MatrixXd MatrixSquareRoot(const Eigen::MatrixXd& mat);

Eigen::MatrixXd MatrixSquareRootForSemidefinitePositiveMat(
    const Eigen::MatrixXd& mat);

}  // namespace gopt

#endif