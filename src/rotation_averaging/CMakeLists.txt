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

OPTIMIZER_ADD_GTEST(lagrange_dual_rotation_estimator_test
  lagrange_dual_rotation_estimator_test.cc)
OPTIMIZER_ADD_GTEST(hybrid_rotation_estimator_test
  hybrid_rotation_estimator_test.cc)
OPTIMIZER_ADD_GTEST(robust_l1l2_rotation_estimator_test
  robust_l1l2_rotation_estimator_test.cc)
