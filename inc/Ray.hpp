#ifndef RAY_HPP
#define RAY_HPP

#include "Math.hpp"
#include "Point.hpp"
#include "Standards.hpp"
#include "Vector.hpp"

namespace RTB {

class Ray {
   private:
    Vec3R m_direction;
    Point3R m_origin;

   public:
    Ray() {
        m_direction = {0, 0, 0};
        m_origin = {0, 0, 0};
    }

    Ray(const Point3R& origin, const Vec3R& direction) {
        m_origin = origin;
        m_direction = direction;
        Normalize();
    };

    void Update(const Point3R& newOrigin, const Vec3R& newDirection) {
        m_origin = newOrigin;
        m_direction = newDirection;
        Normalize();
    }

    void Normalize() {
        m_direction = unit_vector(m_direction);
    }

    Point3R GetPosition(const RESOLUTION t) const {
        return {m_origin[x] + t * m_direction[x],
                m_origin[y] + t * m_direction[y],
                m_origin[z] + t * m_direction[z]};
    }

    Vec3R GetDirection() const {
        return m_direction;
    }
    Point3R GetOrigin() const {
        return m_origin;
    }
};

// using RayR = Ray<RESOLUTION>;

}  // namespace RTB

#endif  // RAY_HPP