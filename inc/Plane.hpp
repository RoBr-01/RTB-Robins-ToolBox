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

    T GetIntersection(const Ray<T, 3>& ray) const;

    void Reflect(Ray<T, 3>& ray, T& t) const;

    Vector<T, 4> GetCoefficients() const {
        return m_coefficients;
    }

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
    m_coefficients.NormalizeInPlace();
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
T Plane<T>::GetIntersection(const Ray<T, 3>& ray) const {
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

// Non-member functions

template <typename T>
void Plane<T>::Reflect(Ray<T, 3>& ray, T& t) const {
    Point<T, 3> intersectionPoint = ray.GetPosition(t);
    Vector<T, 3> normal = GetNormalVector();
    normal.NormalizeInPlace();

    ray.Normalize();
    Vector<T, 3> incomingDirection = ray.GetDirection();

    Vector<T, 3> reflectedDirection =
        incomingDirection -
        normal * (2 * DotProduct(incomingDirection, normal));

    reflectedDirection.NormalizeInPlace();
    ray.Update(intersectionPoint, reflectedDirection);
}

// Intersect two planes in the form:   n · x + d = 0
// Assumes: normals are unit length (planes are pre-normalized).
// Returns: a Ray (point + direction) representing the intersection line.
//          If planes are parallel or degenerate, direction = (0,0,0).

template <typename T>
Ray<T,3> IntersectPlanes(const Plane<T>& p1,
                               const Plane<T>& p2)
{
    Vector<T,3> n1 = p1.GetNormalVector();  // unit normals
    Vector<T,3> n2 = p2.GetNormalVector();
    T d1 = p1.GetCoefficients()[3];
    T d2 = p2.GetCoefficients()[3];

    // Direction of the intersection: n1 × n2
    Vector<T,3> dir = CrossProduct(n1, n2);
    T denom = dir.MagnitudeSquared();

    // If nearly parallel
    if (denom < std::numeric_limits<T>::epsilon()) {
        return Ray<T,3>({0,0,0}, {0,0,0});
    }

    // Choose the coordinate with the largest magnitude for numerical stability
    T ax = std::abs(dir[0]);
    T ay = std::abs(dir[1]);
    T az = std::abs(dir[2]);

    int k = (ax >= ay && ax >= az) ? 0 :
            (ay >= az)            ? 1 : 2;

    // Solve reduced 2×2 system by setting coordinate k = 0
    T x = 0, y = 0, z = 0;

    auto eps = std::numeric_limits<T>::epsilon();

    if (k == 0) {
        // x = 0 → solve for y,z
        T A = n1[1], B = n1[2], C = -d1;
        T D = n2[1], E = n2[2], F = -d2;

        T det = A*E - B*D;
        if (std::abs(det) < eps) {
            return Ray<T,3>({0,0,0}, {0,0,0});
        }

        y = (C*E - B*F) / det;
        z = (A*F - C*D) / det;
    }
    else if (k == 1) {
        // y = 0 → solve for x,z
        T A = n1[0], B = n1[2], C = -d1;
        T D = n2[0], E = n2[2], F = -d2;

        T det = A*E - B*D;
        if (std::abs(det) < eps) {
            return Ray<T,3>({0,0,0}, {0,0,0});
        }

        x = (C*E - B*F) / det;
        z = (A*F - C*D) / det;
    }
    else { // k == 2
        // z = 0 → solve for x,y
        T A = n1[0], B = n1[1], C = -d1;
        T D = n2[0], E = n2[1], F = -d2;

        T det = A*E - B*D;
        if (std::abs(det) < eps) {
            return Ray<T,3>({0,0,0}, {0,0,0});
        }

        x = (C*E - B*F) / det;
        y = (A*F - C*D) / det;
    }

    Point<T,3> p0{x, y, z};

    return Ray<T,3>(p0, dir.Normalize());
}

// template <typename T>
// inline Ray<T, 3> IntersectPlanes(const Plane<T>& plane1,
//                                  const Plane<T>& plane2) {
//     Vector<T, 3> normal1 = plane1.GetNormalVector();
//     Vector<T, 3> normal2 = plane2.GetNormalVector();
//     T d1 = plane1.GetCoefficients()[3];
//     T d2 = plane2.GetCoefficients()[3];

//     // Direction of the intersection line (cross product of normals)
//     Vector<T, 3> direction = CrossProduct(normal1, normal2);

//     // Check if planes are parallel (direction is zero vector)
//     if (direction.Magnitude() < std::numeric_limits<T>::epsilon()) {
//         std::cerr << "Planes are parallel and do not intersect.\n";
//         // Consider throwing an exception or returning an optional here.
//     }

//     // Find a point on the line by fixing one coordinate (z=0, y=0, or x=0)
//     Point<T, 3> point;
//     T denom;

//     // Try z = 0
//     denom = normal1[0] * normal2[1] - normal2[0] * normal1[1];
//     if (std::abs(denom) > std::numeric_limits<T>::epsilon()) {
//         point[0] = (-d1 * normal2[1] - (-d2) * normal1[1]) / denom;
//         point[1] = (normal1[0] * (-d2) - normal2[0] * (-d1)) / denom;
//         point[2] = static_cast<T>(0);
//     }
//     // Try y = 0
//     else if (std::abs(normal1[0] * normal2[2] - normal2[0] * normal1[2]) >
//              std::numeric_limits<T>::epsilon()) {
//         denom = normal1[0] * normal2[2] - normal2[0] * normal1[2];
//         point[0] = (-d1 * normal2[2] - (-d2) * normal1[2]) / denom;
//         point[2] = (normal1[0] * (-d2) - normal2[0] * (-d1)) / denom;
//         point[1] = static_cast<T>(0);
//     }
//     // Try x = 0
//     else {
//         denom = normal1[1] * normal2[2] - normal2[1] * normal1[2];
//         point[1] = (-d1 * normal2[2] - (-d2) * normal1[2]) / denom;
//         point[2] = (normal1[1] * (-d2) - normal2[1] * (-d1)) / denom;
//         point[0] = static_cast<T>(0);
//     }

//     return Ray<T, 3>(point, direction);
// }

// explicit instantiation
template class Plane<float>;
using Planef = Plane<float>;

}  // namespace RTB

#endif /* PLANE_HPP */
