#ifndef TIMER_HPP
#define TIMER_HPP

#include <chrono>
#include <iostream>
#include <string>

// Shamelessly "inspired" by thecherno:
// https://github.com/StudioCherno/Walnut/blob/master/Walnut/src/Walnut/Timer.h

namespace RTB {

class Timer {
 public:
  Timer() { Start(); };

  void Start() { Clock = std::chrono::high_resolution_clock::now(); }

  float Stop_and_count(const std::string& resolution) {
    auto elapsed_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                          std::chrono::high_resolution_clock::now() - Clock)
                          .count();

    if (resolution == "s") {
      return elapsed_ns * 0.000000001f;
    } else if (resolution == "ms") {
      return elapsed_ns * 0.000001f;
    } else if (resolution == "us") {
      return elapsed_ns * 0.001f;
    } else {
      return elapsed_ns;
    }
  }

 private:
  std::chrono::time_point<std::chrono::high_resolution_clock> Clock;
};

class ScopedTimer {
 public:
  ScopedTimer(const std::string& name, const std::string& resolution)
      : Name(name), Resolution(resolution) {}
  ~ScopedTimer() {
    float time = Timer.Stop_and_count(Resolution);
    std::cout << "[TIMER] " << Name << " - " << time << " " << Resolution
              << "\n";
  }

 private:
  std::string Name;
  std::string Resolution;
  Timer Timer;
};
}  // namespace RTB
#endif  // TIMER_HPP