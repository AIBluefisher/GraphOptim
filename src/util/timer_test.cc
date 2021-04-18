#define TEST_NAME "util/timer"
#include "util/timer.h"

#include "util/testing.h"

namespace gopt {

BOOST_AUTO_TEST_CASE(TestDefault) {
  Timer timer;
  BOOST_CHECK_EQUAL(timer.ElapsedMicroSeconds(), 0);
  BOOST_CHECK_EQUAL(timer.ElapsedSeconds(), 0);
  BOOST_CHECK_EQUAL(timer.ElapsedMinutes(), 0);
  BOOST_CHECK_EQUAL(timer.ElapsedHours(), 0);
}

BOOST_AUTO_TEST_CASE(TestStart) {
  Timer timer;
  timer.Start();
  BOOST_CHECK_GE(timer.ElapsedMicroSeconds(), 0);
  BOOST_CHECK_GE(timer.ElapsedSeconds(), 0);
  BOOST_CHECK_GE(timer.ElapsedMinutes(), 0);
  BOOST_CHECK_GE(timer.ElapsedHours(), 0);
}

BOOST_AUTO_TEST_CASE(TestPause) {
  Timer timer;
  timer.Start();
  timer.Pause();
  double prev_time = timer.ElapsedMicroSeconds();
  for (size_t i = 0; i < 1000; ++i) {
    BOOST_CHECK_EQUAL(timer.ElapsedMicroSeconds(), prev_time);
    prev_time = timer.ElapsedMicroSeconds();
  }
  timer.Resume();
  for (size_t i = 0; i < 1000; ++i) {
    BOOST_CHECK_GE(timer.ElapsedMicroSeconds(), prev_time);
  }
  timer.Reset();
  BOOST_CHECK_EQUAL(timer.ElapsedMicroSeconds(), 0);
}

}  // namespace gopt
