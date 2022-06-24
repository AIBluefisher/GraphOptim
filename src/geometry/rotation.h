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
