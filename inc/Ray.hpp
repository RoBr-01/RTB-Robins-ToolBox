#ifndef RAY_HPP
#define RAY_HPP

#include <iostream>

#include "Point.hpp"
#include "Standards.hpp"
#include "Vector.hpp"

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
    Ray() = default;

    Ray(const Point<T, N>& origin, const Vector<T, N>& direction);

    /**
     * @brief Sets a new origin and direction in one call.
     */
    void Set(const Point<T, N>& newOrigin, const Vector<T, N>& newDirection);

    /**
     * @brief Normalizes the direction vector in-place.
     */
    void Normalize();

    /**
     * @brief Returns the point along the ray at parameter t: origin + t *
     * direction.
     */
    Point<T, N> GetPosition(T t) const;

    const Point<T, N>& GetOrigin() const {
        return m_origin;
    }
    const Vector<T, N>& GetDirection() const {
        return m_direction;
    }

    void Print() const;

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
void Ray<T, N>::Set(const Point<T, N>& newOrigin,
                    const Vector<T, N>& newDirection) {
    m_origin = newOrigin;
    m_direction = newDirection;
}

template <typename T, std::size_t N>
void Ray<T, N>::Normalize() {
    m_direction.NormalizeInPlace();
}

template <typename T, std::size_t N>
Point<T, N> Ray<T, N>::GetPosition(T t) const {
    Point<T, N> position;
    for (std::size_t i = 0; i < N; ++i)
        position[i] = m_origin[i] + t * m_direction[i];
    return position;
}

template <typename T, std::size_t N>
void Ray<T, N>::Print() const {
    std::cout << "Origin:    ";
    m_origin.Print();
    std::cout << "Direction: ";
    m_direction.Print();
}

// ==============================
// Convenience type aliases
// ==============================

using Ray3f = Ray<float, 3>;

}  // namespace RTB

#endif  // RAY_HPP