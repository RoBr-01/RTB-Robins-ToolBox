#ifndef PLANE_HPP
#define PLANE_HPP

#include <array>

#include "Point.hpp"
#include "Ray.hpp"
#include "Standards.hpp"
#include "Vector.hpp"

namespace RTB {

template <typename T>
class Plane {
   public:
    Plane() : m_coefficients{0, 0, 0, 0} {}
    Plane(T a, T b, T c, T d) : m_coefficients{a, b, c, d} {};

    Plane(Point3R A, Point3R B, Point3R C) {
        m_coefficients[0] =
            (B[1] - A[1]) * (C[2] - A[2]) - (C[1] - A[1]) * (B[2] - A[2]);
        m_coefficients[1] =
            (B[2] - A[2]) * (C[0] - A[0]) - (C[2] - A[2]) * (B[0] - A[0]);
        m_coefficients[2] =
            (B[0] - A[0]) * (C[1] - A[1]) - (C[0] - A[0]) * (B[1] - A[1]);
        m_coefficients[3] =
            -(m_coefficients[0] * A[0] + m_coefficients[1] * A[1] +
              m_coefficients[2] * A[2]);
    }

    void Invert() {
        m_coefficients[0] = -m_coefficients[0];
        m_coefficients[1] = -m_coefficients[1];
        m_coefficients[2] = -m_coefficients[2];
        m_coefficients[3] = -m_coefficients[3];
    }

    void Normalize() {
        m_coefficients = unit_vector(m_coefficients);
    }

    void Print() const {
        std::cout << "Plane coefficients: ";
        m_coefficients.print();  // Call the print method of the Vector class
    }

    Vec3R GetNormalVector() {
        Vec3R normal;
        normal[0] = m_coefficients[0];
        normal[1] = m_coefficients[1];
        normal[2] = m_coefficients[2];

        normal = unit_vector(normal);
        return normal;
    }

    T GetIntersection(const Ray &ray) {
        Point3R rayOriginPoint = ray.GetOrigin();
        Vec3R rayOrigin{
            rayOriginPoint[0], rayOriginPoint[1], rayOriginPoint[2]};
        Vec3R rayDirection = ray.GetDirection();
        T denominator = m_coefficients[0] * rayDirection[0] +
                        m_coefficients[1] * rayDirection[1] +
                        m_coefficients[2] * rayDirection[2];

        if (std::abs(denominator) < std::numeric_limits<T>::epsilon()) {
            // The ray is parallel to the plane
            return -1;
        }

        T numerator = -(m_coefficients[0] * rayOrigin[0] +
                        m_coefficients[1] * rayOrigin[1] +
                        m_coefficients[2] * rayOrigin[2] + m_coefficients[3]);

        T t = numerator / denominator;

        // Return -1 if t is negative
        if (t < 0) {
            return -1;
        }

        return t;
    }

    void Reflect(Ray &ray, float &t) {
        Point3R intersectionPoint = ray.GetPosition(t);
        Vec3R normal = GetNormalVector();
        normal = unit_vector(normal);

        ray.Normalize();
        Vec3R incomingDirection = ray.GetDirection();

        Vec3R reflectedDirection =
            incomingDirection -
            normal * (2 * dotprod(incomingDirection, normal));

        reflectedDirection = unit_vector(reflectedDirection);
        ray.Update(intersectionPoint, reflectedDirection);
    }

   private:
    Vector<T, 4> m_coefficients;  // Because member functions are hidden anyway
};

using PlaneR = Plane<RESOLUTION>;
}  // namespace RTB

#endif  // PLANE_HPP