#ifndef RAY_HPP
#define RAY_HPP

#include "Point.hpp"
#include "Standards.hpp"
#include "Vector.hpp"

namespace RTB {

// Interface
template <typename T, std::size_t N>
class Ray {
   public:
    Ray() = default;

    Ray(const Point<T, N>& origin, const Vector<T, N>& direction);

    void Update(const Point<T, N>& newOrigin, const Vector<T, N>& newDirection);

    void Normalize();

    Point<T, N> GetPosition(const T& t) const;

    Point<T, N> GetOrigin() const;

    Vector<T, N> GetDirection() const;

    void Print() const;

   private:
    Point<T, N> m_origin;
    Vector<T, N> m_direction;
};

// Implementation
template <typename T, std::size_t N>
Ray<T, N>::Ray(const Point<T, N>& origin, const Vector<T, N>& direction)
    : m_origin(origin), m_direction(direction) {}

template <typename T, std::size_t N>
void Ray<T, N>::Update(const Point<T, N>& newOrigin,
                       const Vector<T, N>& newDirection) {
    m_origin = newOrigin;
    m_direction = newDirection;
}

template <typename T, std::size_t N>
void Ray<T, N>::Normalize() {
    m_direction = unit_vector(m_direction);
}

template <typename T, std::size_t N>
Point<T, N> Ray<T, N>::GetPosition(const T& t) const {
    Point<T, N> position;
    for (std::size_t i = 0; i < N; ++i) {
        position[i] = m_origin[i] + t * m_direction[i];
    }
    return position;
}

template <typename T, std::size_t N>
Point<T, N> Ray<T, N>::GetOrigin() const {
    return m_origin;
}

template <typename T, std::size_t N>
Vector<T, N> Ray<T, N>::GetDirection() const {
    return m_direction;
}

// Print vector components
template <typename T, std::size_t N>
void Ray<T, N>::Print() const {
    std::cout << "Origin: ";
    m_origin.Print();
    std::cout << "Direction: ";
    m_direction.Print();
}

// Explicit instantiation
template class Ray<RESOLUTION, 3>;
using Ray3R = Ray<RESOLUTION, 3>;

}  // namespace RTB

#endif  // RAY_HPP