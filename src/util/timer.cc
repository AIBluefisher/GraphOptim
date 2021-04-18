#include "util/timer.h"

#include <glog/logging.h>

using namespace std::chrono;

namespace gopt {

Timer::Timer() : started_(false), paused_(false) {}

void Timer::Start() {
  started_ = true;
  paused_ = false;
  start_time_ = high_resolution_clock::now();
}

void Timer::Restart() {
  started_ = false;
  Start();
}

void Timer::Pause() {
  paused_ = true;
  pause_time_ = high_resolution_clock::now();
}

void Timer::Resume() {
  paused_ = false;
  start_time_ += high_resolution_clock::now() - pause_time_;
}

void Timer::Reset() {
  started_ = false;
  paused_ = false;
}

double Timer::ElapsedMicroSeconds() const {
  if (!started_) {
    return 0.0;
  }
  if (paused_) {
    return duration_cast<microseconds>(pause_time_ - start_time_).count();
  } else {
    return duration_cast<microseconds>(high_resolution_clock::now() -
                                       start_time_)
        .count();
  }
}

double Timer::ElapsedSeconds() const { return ElapsedMicroSeconds() / 1e6; }

double Timer::ElapsedMinutes() const { return ElapsedSeconds() / 60; }

double Timer::ElapsedHours() const { return ElapsedMinutes() / 60; }

void Timer::PrintSeconds() const {
  LOG(INFO) << "Elapsed time: " << ElapsedSeconds() << " [seconds]";
}

void Timer::PrintMinutes() const {
  LOG(INFO) << "Elapsed time: " << ElapsedMinutes() << " [minutes]";
}

void Timer::PrintHours() const {
  LOG(INFO) << "Elapsed time: " << ElapsedHours() << " [hours]";
}

}  // namespace gopt
