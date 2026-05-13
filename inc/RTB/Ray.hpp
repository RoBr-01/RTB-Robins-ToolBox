#ifndef RAY_HPP
#define RAY_HPP

// STL
#include <cstddef>

// LOCAL
#include <RTB/Point.hpp>
#include <RTB/Vector.hpp>
#include <iostream>

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
    void set(const Point<T, N>& newOrigin, const Vector<T, N>& newDirection);

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
void Ray<T, N>::set(const Point<T, N>& newOrigin,
                    const Vector<T, N>& newDirection) {
    m_origin = newOrigin;
    m_direction = newDirection;
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

// ==============================
// Convenience type aliases
// ==============================

using Ray3f = Ray<float, 3>;

}  // namespace RTB

#endif  // RAY_HPP