#ifndef RAY_HPP
#define RAY_HPP

#include "Point.hpp"
#include "Standards.hpp"
#include "Vector.hpp"

namespace RTB {

template <typename T, std::size_t N>
class Ray {
   public:
    Ray() = default;

    Ray(const Point<T, N>& origin, const Vector<T, N>& direction) {
        m_origin = origin;
        m_direction = direction;
    };

    void Update(const Point<T, N>& newOrigin,
                const Vector<T, N>& newDirection) {
        m_origin = newOrigin;
        m_direction = newDirection;
    }

    void Normalize() {
        m_direction = unit_vector(m_direction);
    }

    Point<T, N> GetPosition(const T& t) const {
        Point<T, N> position;
        for (std::size_t i = 0; i < N; ++i) {
            position[i] = m_origin[i] + t * m_direction[i];
        }
        return position;
    }

    Vector<T, N> GetDirection() const {
        return m_direction;
    }
    Point<T, N> GetOrigin() const {
        return m_origin;
    }

   private:
    Vector<T, N> m_direction;
    Point<T, N> m_origin;
};

template class Ray<RESOLUTION, 3>;
using Ray3R = Ray<RESOLUTION, 3>;

}  // namespace RTB

#endif  // RAY_HPP