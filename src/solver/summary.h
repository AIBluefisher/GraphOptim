#ifndef SOLVER_SUMMARY_H_
#define SOLVER_SUMMARY_H_

#include <chrono>
#include <iostream>

namespace gopt {
namespace solver {

struct Summary {
  unsigned total_iterations_num = 0;

  std::chrono::high_resolution_clock::time_point begin_time;
  std::chrono::high_resolution_clock::time_point end_time;
  std::chrono::high_resolution_clock::time_point prev_time;

  Summary() { total_iterations_num = 0; }

  Summary(const Summary& summary) {
    total_iterations_num = summary.total_iterations_num;
    begin_time = summary.begin_time;
    end_time = summary.end_time;
  }

  double TotalTime() {
    return std::chrono::duration_cast<std::chrono::duration<int, std::milli>>(
               end_time - begin_time)
        .count();
  }

  double Duration() {
    if (total_iterations_num == 1) {
      prev_time = begin_time;
    } else {
      prev_time = end_time;
    }
    end_time = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::duration<int, std::milli>>(
               end_time - prev_time)
        .count();
  }

  void Report() {
    std::cout << "\nLagrange Dual Rotation Averaging Report: \n";
    std::cout << "Total iterations: " << total_iterations_num << std::endl;
    std::cout << "Time took: " << TotalTime() << " milliseconds\n";
  }
};
}  // namespace solver
}  // namespace gopt

#endif  // SOLVER_SUMMARY_H_
