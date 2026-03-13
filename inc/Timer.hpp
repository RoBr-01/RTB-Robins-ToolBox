#ifndef TIMER_HPP
#define TIMER_HPP

#include <chrono>
#include <iostream>
#include <string>

// Original by the cherno:
// https://github.com/StudioCherno/Walnut/blob/master/Walnut/src/Walnut/Timer.h

namespace RTB {

namespace detail {

// Maps a std::chrono duration type to its display label.
template <typename Duration>
constexpr const char* DurationLabel() {
    return "ns";
}
template <>
constexpr const char* DurationLabel<std::chrono::seconds>() {
    return "s";
}
template <>
constexpr const char* DurationLabel<std::chrono::milliseconds>() {
    return "ms";
}
template <>
constexpr const char* DurationLabel<std::chrono::microseconds>() {
    return "us";
}
template <>
constexpr const char* DurationLabel<std::chrono::nanoseconds>() {
    return "ns";
}

}  // namespace detail

class Timer {
   public:
    Timer() {
        Start();
    }

    void Start() {
        m_start = std::chrono::high_resolution_clock::now();
    }

    /**
     * @brief Returns elapsed time since the last Start() call.
     *
     * @tparam Duration  A std::chrono duration type. Defaults to milliseconds.
     * @return           Elapsed time as a double in the requested unit.
     *
     * Example:
     *   timer.Elapsed();                              // milliseconds (default)
     *   timer.Elapsed<std::chrono::microseconds>();
     *   timer.Elapsed<std::chrono::seconds>();
     */
    template <typename Duration = std::chrono::milliseconds>
    double Elapsed() const {
        return std::chrono::duration<double, typename Duration::period>(
                   std::chrono::high_resolution_clock::now() - m_start)
            .count();
    }

    /**
     * @brief Prints elapsed time to std::cout.
     *
     * @tparam Duration  A std::chrono duration type. Defaults to milliseconds.
     * @param  label     Descriptive label printed alongside the time.
     */
    template <typename Duration = std::chrono::milliseconds>
    void Print(const std::string& label = "") const {
        std::cout << "[TIMER] " << label << " - " << Elapsed<Duration>() << " "
                  << detail::DurationLabel<Duration>() << "\n";
    }

   private:
    std::chrono::time_point<std::chrono::high_resolution_clock> m_start;
};

/**
 * @brief RAII timer that prints elapsed time on scope exit.
 *
 * @tparam Duration  A std::chrono duration type. Defaults to milliseconds.
 *
 * Example:
 *   ScopedTimer<std::chrono::microseconds> t("my label");
 */
template <typename Duration = std::chrono::milliseconds>
class ScopedTimer {
   public:
    explicit ScopedTimer(const std::string& name) : m_name(name) {}

    ~ScopedTimer() {
        m_timer.Print<Duration>(m_name);
    }

    ScopedTimer(const ScopedTimer&) = delete;
    ScopedTimer& operator=(const ScopedTimer&) = delete;

   private:
    std::string m_name;
    Timer m_timer;
};

}  // namespace RTB

#endif  // TIMER_HPP