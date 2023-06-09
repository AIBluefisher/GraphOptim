# BSD 3-Clause License

# Copyright (c) 2021, Chenyu
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

OPTIMIZER_ADD_HEADERS(
  hybrid_rotation_estimator.h
  irls_rotation_local_refiner.h
  l1_rotation_global_estimator.h
  lagrange_dual_rotation_estimator.h
  robust_l1l2_rotation_estimator.h)

OPTIMIZER_ADD_SOURCES(
  hybrid_rotation_estimator.cc
  irls_rotation_local_refiner.cc
  l1_rotation_global_estimator.cc
  lagrange_dual_rotation_estimator.cc
  robust_l1l2_rotation_estimator.cc)

if(TESTS_ENABLED)
  OPTIMIZER_ADD_GTEST(lagrange_dual_rotation_estimator_test
    lagrange_dual_rotation_estimator_test.cc)
  OPTIMIZER_ADD_GTEST(hybrid_rotation_estimator_test
    hybrid_rotation_estimator_test.cc)
  OPTIMIZER_ADD_GTEST(robust_l1l2_rotation_estimator_test
    robust_l1l2_rotation_estimator_test.cc)
endif()
