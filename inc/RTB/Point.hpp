/*
 ____ _____ ____
|  _ \_   _| __ )
| |_) || | |  _ \
|  _ < | | | |_) |
|_| \_\|_| |____/

 Point.hpp
 A fixed-size point in N-dimensional space.

 Copyright (c) 2026 Robin Breugelmans
 Licensed under the MIT License. See LICENSE in the project root.
*/

#ifndef RTB_POINT_HPP
#define RTB_POINT_HPP

// STL
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <iostream>
#include <type_traits>

namespace RTB {

// ==============================
// Forward declarations
// ==============================

template <typename T, std::size_t N>
class Point;

template <typename T1, typename T2, std::size_t N>
[[nodiscard]] auto distance2Points(const Point<T1, N>& point_1,
                                   const Point<T2, N>& point_2)
    -> std::common_type_t<T1, T2>;

template <typename T1, typename T2, std::size_t N>
[[nodiscard]] auto midpoint2Points(const Point<T1, N>& point_1,
                                   const Point<T2, N>& point_2)
    -> Point<std::common_type_t<T1, T2>, N>;

template <typename T, std::size_t N>
std::ostream& operator<<(std::ostream& out, const Point<T, N>& point);

template <typename T, std::size_t N>
bool operator==(const Point<T, N>& left_point, const Point<T, N>& right_point);

template <typename T, std::size_t N>
bool operator!=(const Point<T, N>& left_point, const Point<T, N>& right_point);

// ==============================
// Point Class
// ==============================

/**
 * @brief A fixed-size point in N-dimensional space.
 *
 * @tparam T Scalar type. Must be arithmetic.
 * @tparam N Dimension. Must be greater than zero.
 */
template <typename T, std::size_t N>
class Point {
    static_assert(std::is_arithmetic_v<T>,
                  "Point<T, N>: T must be an arithmetic type.");
    static_assert(N > 0, "Point<T, N>: N must be greater than zero.");

   public:
    Point() : m_coords{} {}

    Point(const std::initializer_list<T>& values) : m_coords{} {
        assert(values.size() == N);
        std::copy(values.begin(), values.end(), m_coords.begin());
    }

    template <typename... Args,
              typename = std::enable_if_t<
                  (sizeof...(Args) == N) &&
                  (std::is_arithmetic_v<std::decay_t<Args>> && ...)>>
    explicit Point(Args&&... args)
        : m_coords{static_cast<T>(std::forward<Args>(args))...} {}

    explicit Point(const std::array<T, N>& values) : m_coords(values) {}

    [[nodiscard]] std::array<T, N> getCoords() const {
        return m_coords;
    }

    void setCoords(const std::array<T, N>& new_coords) {
        m_coords = new_coords;
    }

    T& operator[](std::size_t index) {
        assert(index < N);
        return m_coords[index];
    }

    const T& operator[](std::size_t index) const {
        assert(index < N);
        return m_coords[index];
    }

    /** @brief Prints the point to std::cout. */
    void print() const {
        std::cout << "[Point] - " << *this << "\n";
    }

   private:
    std::array<T, N> m_coords;
};

// ==============================
// Non-member functions
// ==============================

/**
 * @brief Returns the Euclidean distance between two points.
 */
template <typename T1, typename T2, std::size_t N>
[[nodiscard]] auto distance2Points(const Point<T1, N>& point_1,
                                   const Point<T2, N>& point_2)
    -> std::common_type_t<T1, T2> {
    using R = std::common_type_t<T1, T2>;
    R sum = R{0};
    for (std::size_t i = 0; i < N; ++i) {
        const R diff = static_cast<R>(point_2[i]) - static_cast<R>(point_1[i]);
        sum += diff * diff;
    }
    return std::sqrt(sum);
}

/**
 * @brief Returns the midpoint between two points.
 */
template <typename T1, typename T2, std::size_t N>
[[nodiscard]] auto midpoint2Points(const Point<T1, N>& point_1,
                                   const Point<T2, N>& point_2)
    -> Point<std::common_type_t<T1, T2>, N> {
    using R = std::common_type_t<T1, T2>;
    Point<R, N> mid;
    for (std::size_t i = 0; i < N; ++i) {
        mid[i] = (static_cast<R>(point_1[i]) + static_cast<R>(point_2[i])) /
                 static_cast<R>(2);
    }
    return mid;
}

template <typename T, std::size_t N>
std::ostream& operator<<(std::ostream& out, const Point<T, N>& point) {
    out << "(";
    for (std::size_t i = 0; i < N; ++i) {
        out << point[i];
        if (i < N - 1) {
            out << ", ";
        }
    }
    out << ")";
    return out;
}

template <typename T, std::size_t N>
bool operator==(const Point<T, N>& left_point, const Point<T, N>& right_point) {
    return left_point.getCoords() == right_point.getCoords();
}

template <typename T, std::size_t N>
bool operator!=(const Point<T, N>& left_point, const Point<T, N>& right_point) {
    return !(left_point == right_point);
}

}  // namespace RTB

#endif  // RTB_POINT_HPP
