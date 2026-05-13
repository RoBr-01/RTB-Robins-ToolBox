#ifndef POINT_HPP
#define POINT_HPP

// STL
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <iostream>
#include <type_traits>

namespace RTB {

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

    Point(const std::initializer_list<T>& values) {
        assert(values.size() == N);
        std::copy(values.begin(), values.end(), m_coords.begin());
    }

    [[nodiscard]] std::array<T, N> getCoords() const {
        return m_coords;
    }

    void setCoords(const std::array<T, N>& newCoords) {
        m_coords = newCoords;
    }

    T& operator[](std::size_t index) {
        assert(index < N);
        return m_coords[index];
    }

    const T& operator[](std::size_t index) const {
        assert(index < N);
        return m_coords[index];
    }

    void print() const {
        std::cout << "(";
        for (std::size_t i = 0; i < N; ++i) {
            std::cout << m_coords[i];
            if (i < N - 1) {
                std::cout << ", ";
            }
        }
        std::cout << ")\n";
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
auto distance2Points(const Point<T1, N>& Point1, const Point<T2, N>& Point2)
    -> std::common_type_t<T1, T2> {
    using R = std::common_type_t<T1, T2>;
    R sum = R{0};
    for (std::size_t i = 0; i < N; ++i) {
        R diff = static_cast<R>(Point2[i]) - static_cast<R>(Point1[i]);
        sum += diff * diff;
    }
    return std::sqrt(sum);
}

/**
 * @brief Returns the midpoint between two points.
 */
template <typename T1, typename T2, std::size_t N>
auto midpoint2Points(const Point<T1, N>& Point1, const Point<T2, N>& Point2)
    -> Point<std::common_type_t<T1, T2>, N> {
    using R = std::common_type_t<T1, T2>;
    Point<R, N> mid;
    for (std::size_t i = 0; i < N; ++i) {
        mid[i] = (static_cast<R>(Point1[i]) + static_cast<R>(Point2[i])) /
                 static_cast<R>(2);
    }
    return mid;
}

}  // namespace RTB

#endif  // POINT_HPP
