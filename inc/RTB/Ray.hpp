/*
 ____ _____ ____
|  _ \_   _| __ )
| |_) || | |  _ \
|  _ < | | | |_) |
|_| \_\|_| |____/

 Ray.hpp
 A ray defined by an origin point and a direction vector.

 Copyright (c) 2026 Robin Breugelmans
 Licensed under the MIT License. See LICENSE in the project root.
*/

#ifndef RTB_RAY_HPP
#define RTB_RAY_HPP

// STL
#include <cstddef>
#include <iostream>

// LOCAL
#include <RTB/Point.hpp>
#include <RTB/Vector.hpp>

namespace RTB {

/**
 * @brief A ray defined by an origin point and a direction vector.
 *
 * @tparam T Scalar type.
 * @tparam N Dimension.
 */
template <typename T, std::size_t N>
class Ray {
   public:
    /**
     * @brief Constructs a degenerate ray at the origin with a zero-length
     * direction.
     *
     * A zero-length direction is meaningless for intersection/reflection
     * use. normalize() leaves a zero-length direction unchanged rather
     * than dividing by zero (see Vector::normalizeInPlace()), so calling
     * it on a default-constructed Ray is a silent no-op.
     */
    Ray() = default;

    Ray(const Point<T, N>& origin, const Vector<T, N>& direction);

    /**
     * @brief Sets a new origin and direction in one call.
     */
    void set(const Point<T, N>& new_origin, const Vector<T, N>& new_direction);

    /**
     * @brief Normalizes the direction vector in-place.
     */
    void normalize();

    /**
     * @brief Returns the point along the ray at parameter t: origin + t *
     * direction.
     */
    [[nodiscard]] Point<T, N> getPosition(T step) const;

    [[nodiscard]] const Point<T, N>& getOrigin() const {
        return m_origin;
    }

    [[nodiscard]] const Vector<T, N>& getDirection() const {
        return m_direction;
    }

    void print() const;

   private:
    Point<T, N> m_origin;
    Vector<T, N> m_direction;
};

// ==============================
// Implementation
// ==============================

template <typename T, std::size_t N>
Ray<T, N>::Ray(const Point<T, N>& origin, const Vector<T, N>& direction)
    : m_origin(origin), m_direction(direction) {}

template <typename T, std::size_t N>
void Ray<T, N>::set(const Point<T, N>& new_origin,
                    const Vector<T, N>& new_direction) {
    m_origin = new_origin;
    m_direction = new_direction;
}

template <typename T, std::size_t N>
void Ray<T, N>::normalize() {
    m_direction.normalizeInPlace();
}

template <typename T, std::size_t N>
Point<T, N> Ray<T, N>::getPosition(T step) const {
    Point<T, N> position;
    for (std::size_t i = 0; i < N; ++i) {
        position[i] = m_origin[i] + step * m_direction[i];
    }
    return position;
}

template <typename T, std::size_t N>
void Ray<T, N>::print() const {
    std::cout << "Origin:    ";
    m_origin.print();
    std::cout << "Direction: ";
    m_direction.print();
}

}  // namespace RTB

#endif  // RTB_RAY_HPP