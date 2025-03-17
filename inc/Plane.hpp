#ifndef PLANE_HPP
#define PLANE_HPP

// STL
#include <array>

// RTB
#include "Point.hpp"
#include "Ray.hpp"
#include "Standards.hpp"
#include "Vector.hpp"

namespace RTB {

// Interface
template <typename T>
class Plane {
   public:
    Plane();

    Plane(T a, T b, T c, T d);

    Plane(Point<T, 3> A, Point<T, 3> B, Point<T, 3> C);

    void Invert();

    void Normalize();

    void Print() const;

    Vector<T, 3> GetNormalVector() const;

    T GetIntersection(const Ray<T, 3> &ray) const;

    void Reflect(Ray<T, 3> &ray, T &t) const;

   private:
    Vector<T, 4> m_coefficients;
    // Would this be better using a normal array?
};

template <typename T>
Plane<T>::Plane() : m_coefficients{0, 0, 0, 0} {};

template <typename T>
Plane<T>::Plane(T a, T b, T c, T d) : m_coefficients{a, b, c, d} {};

template <typename T>
Plane<T>::Plane(Point<T, 3> A, Point<T, 3> B, Point<T, 3> C) {
    m_coefficients[0] =
        (B[1] - A[1]) * (C[2] - A[2]) - (C[1] - A[1]) * (B[2] - A[2]);
    m_coefficients[1] =
        (B[2] - A[2]) * (C[0] - A[0]) - (C[2] - A[2]) * (B[0] - A[0]);
    m_coefficients[2] =
        (B[0] - A[0]) * (C[1] - A[1]) - (C[0] - A[0]) * (B[1] - A[1]);
    m_coefficients[3] = -(m_coefficients[0] * A[0] + m_coefficients[1] * A[1] +
                          m_coefficients[2] * A[2]);
}

template <typename T>
void Plane<T>::Invert() {
    m_coefficients[0] = -m_coefficients[0];
    m_coefficients[1] = -m_coefficients[1];
    m_coefficients[2] = -m_coefficients[2];
    m_coefficients[3] = -m_coefficients[3];
}

template <typename T>
void Plane<T>::Normalize() {
    m_coefficients = unit_vector(m_coefficients);
}

template <typename T>
void Plane<T>::Print() const {
    std::cout << "Plane coefficients: ";
    m_coefficients.Print();  // Call the print method of the Vector class
}

template <typename T>
Vector<T, 3> Plane<T>::GetNormalVector() const {
    Vector<T, 3> normal;
    normal[0] = m_coefficients[0];
    normal[1] = m_coefficients[1];
    normal[2] = m_coefficients[2];

    // normal = unit_vector(normal);  // Shouldnt be here
    return normal;
}

template <typename T>
T Plane<T>::GetIntersection(const Ray<T, 3> &ray) const {
    Point<T, 3> rayOriginPoint = ray.GetOrigin();
    Vector<T, 3> rayOrigin{
        rayOriginPoint[0], rayOriginPoint[1], rayOriginPoint[2]};
    Vector<T, 3> rayDirection = ray.GetDirection();
    T denominator = m_coefficients[0] * rayDirection[0] +
                    m_coefficients[1] * rayDirection[1] +
                    m_coefficients[2] * rayDirection[2];

    if (std::abs(denominator) < std::numeric_limits<T>::epsilon()) {
        // The ray is parallel to the plane
        return -1;
    }

    T numerator =
        -(m_coefficients[0] * rayOrigin[0] + m_coefficients[1] * rayOrigin[1] +
          m_coefficients[2] * rayOrigin[2] + m_coefficients[3]);

    T t = numerator / denominator;

    // Return -1 if t is negative
    if (t < 0) {
        return -1;
    }

    return t;
}

template <typename T>
void Plane<T>::Reflect(Ray<T, 3> &ray, T &t) const {
    Point<T, 3> intersectionPoint = ray.GetPosition(t);
    Vector<T, 3> normal = GetNormalVector();
    normal = unit_vector(normal);

    ray.Normalize();
    Vector<T, 3> incomingDirection = ray.GetDirection();

    Vector<T, 3> reflectedDirection =
        incomingDirection - normal * (2 * dotprod(incomingDirection, normal));

    reflectedDirection = unit_vector(reflectedDirection);
    ray.Update(intersectionPoint, reflectedDirection);
}

// explicit instantiation
template class Plane<RESOLUTION>;
using PlaneR = Plane<RESOLUTION>;
}  // namespace RTB

#endif  // PLANE_HPP