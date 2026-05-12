#ifndef POINT_HPP
#define POINT_HPP

// STL
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
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
    static_assert(std::is_arithmetic<T>::value,
                  "Point<T, N>: T must be an arithmetic type.");
    static_assert(N > 0, "Point<T, N>: N must be greater than zero.");

   public:
    Point() : m_coords{} {}

    Point(const std::initializer_list<T>& values) {
        assert(values.size() == N);
        std::copy(values.begin(), values.end(), m_coords.begin());
    }

    std::array<T, N> GetCoords() const {
        return m_coords;
    }

    void SetCoords(const std::array<T, N>& newCoords) {
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

    void Print() const {
        std::cout << "(";
        for (std::size_t i = 0; i < N; ++i) {
            std::cout << m_coords[i];
            if (i < N - 1)
                std::cout << ", ";
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
auto Distance(const Point<T1, N>& P1, const Point<T2, N>& P2) ->
    typename std::common_type<T1, T2>::type {
    using R = typename std::common_type<T1, T2>::type;
    R sum = R{0};
    for (std::size_t i = 0; i < N; ++i) {
        R diff = static_cast<R>(P2[i]) - static_cast<R>(P1[i]);
        sum += diff * diff;
    }
    return std::sqrt(sum);
}

/**
 * @brief Returns the midpoint between two points.
 */
template <typename T1, typename T2, std::size_t N>
auto Midpoint(const Point<T1, N>& P1, const Point<T2, N>& P2)
    -> Point<typename std::common_type<T1, T2>::type, N> {
    using R = typename std::common_type<T1, T2>::type;
    Point<R, N> mid;
    for (std::size_t i = 0; i < N; ++i)
        mid[i] =
            (static_cast<R>(P1[i]) + static_cast<R>(P2[i])) / static_cast<R>(2);
    return mid;
}

// ==============================
// Convenience type aliases
// ==============================

using Point3f = Point<float, 3>;

}  // namespace RTB

#endif  // POINT_HPP