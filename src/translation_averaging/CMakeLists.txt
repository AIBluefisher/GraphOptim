OPTIMIZER_ADD_HEADERS(
  position_estimator.h
  lud_position_estimator.h)

OPTIMIZER_ADD_SOURCES(
  lud_position_estimator.cc)

OPTIMIZER_ADD_GTEST(lud_position_estimator_test
  lud_position_estimator_test.cc)
