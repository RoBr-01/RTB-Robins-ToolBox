/*
 ____ _____ ____
|  _ \_   _| __ )
| |_) || | |  _ \
|  _ < | | | |_) |
|_| \_\|_| |____/

 Timer.hpp
 Simple stopwatch and scoped RAII timer.

 Copyright (c) 2026 Robin Breugelmans
 Licensed under the MIT License. See LICENSE in the project root.
*/

#ifndef RTB_TIMER_HPP
#define RTB_TIMER_HPP

#include <chrono>
#include <iostream>
#include <string>
#include <utility>

// Original by the cherno:
// https://github.com/StudioCherno/Walnut/blob/master/Walnut/src/Walnut/Timer.h

namespace RTB {

namespace detail {

// Maps a std::chrono duration type to its display label.
template <typename Duration>
constexpr const char* durationLabel() {
    return "ns";
}
template <>
constexpr const char* durationLabel<std::chrono::seconds>() {
    return "s";
}
template <>
constexpr const char* durationLabel<std::chrono::milliseconds>() {
    return "ms";
}
template <>
constexpr const char* durationLabel<std::chrono::microseconds>() {
    return "us";
}
template <>
constexpr const char* durationLabel<std::chrono::nanoseconds>() {
    return "ns";
}

}  // namespace detail

class Timer {
   public:
    Timer() {
        start();
    }

    void start() {
        m_start = std::chrono::steady_clock::now();
    }

    /**
     * @brief Returns elapsed time since the last start() call.
     *
     * @tparam Duration  A std::chrono duration type. Defaults to milliseconds.
     * @return           elapsed time as a double in the requested unit.
     *
     * Example:
     *   timer.elapsed();                              // milliseconds (default)
     *   timer.elapsed<std::chrono::microseconds>();
     *   timer.elapsed<std::chrono::seconds>();
     */
    template <typename Duration = std::chrono::nanoseconds>
    [[nodiscard]] double elapsed() const {
        return std::chrono::duration<double, typename Duration::period>(
                   std::chrono::steady_clock::now() - m_start)
            .count();
    }

    /**
     * @brief Prints elapsed time to std::cout.
     *
     * @tparam Duration  A std::chrono duration type. Defaults to milliseconds.
     * @param  label     Descriptive label printed alongside the time.
     */
    template <typename Duration = std::chrono::milliseconds>
    void print(const std::string& label = "") const {
        std::cout << "[TIMER] " << label << " - " << elapsed<Duration>() << " "
                  << detail::durationLabel<Duration>() << "\n";
    }

   private:
    std::chrono::time_point<std::chrono::steady_clock> m_start;
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
    explicit ScopedTimer(std::string name) : m_name(std::move(name)) {}

    ~ScopedTimer() {
        m_timer.print<Duration>(m_name);
    }

    ScopedTimer(const ScopedTimer&) = delete;
    ScopedTimer& operator=(const ScopedTimer&) = delete;

   private:
    std::string m_name;
    Timer m_timer;
};

}  // namespace RTB

#endif  // RTB_TIMER_HPP