#ifndef PLANE_HPP
#define PLANE_HPP

// STL
#include <array>
#include <iostream>
#include <limits>
#include <optional>

// RTB
#include <RTB/Point.hpp>
#include <RTB/Ray.hpp>
#include <RTB/Vector.hpp>

namespace RTB {

/**
 * @brief Infinite plane represented by the equation ax + by + cz + d = 0.
 *
 * Coefficients are stored as {a, b, c, d}. Call Normalize() to ensure
 * the normal vector (a, b, c) is unit length, which is required by
 * GetIntersection(), Reflect(), and IntersectPlanes().
 */
template <typename T>
class Plane {
   public:
    Plane();
    Plane(T a, T b, T c, T d);
    Plane(const Point<T, 3>& A, const Point<T, 3>& B, const Point<T, 3>& C);

    /**
     * @brief Flips the plane normal (negates all coefficients).
     */
    void Invert();

    /**
     * @brief Normalizes the plane so that (a, b, c) is a unit vector.
     *
     * Also scales d accordingly. Required before using GetIntersection(),
     * Reflect(), or IntersectPlanes().
     */
    void Normalize();

    void Print() const;

    /**
     * @brief Returns the normal vector (a, b, c). Not guaranteed unit length
     *        unless Normalize() has been called.
     */
    Vector<T, 3> GetNormalVector() const;

    /**
     * @brief Returns the ray parameter t at which the ray intersects the plane.
     *
     * Returns std::nullopt if the ray is parallel to the plane or the
     * intersection is behind the ray origin (t < 0).
     *
     * Assumes the plane is normalized.
     */
    std::optional<T> GetIntersection(const Ray<T, 3>& ray) const;

    /**
     * @brief Reflects the ray about the plane at parameter t.
     *
     * Assumes the plane is normalized.
     */
    void Reflect(Ray<T, 3>& ray, T t) const;

    std::array<T, 4> GetCoefficients() const {
        return m_coefficients;
    }

   private:
    std::array<T, 4> m_coefficients;
};

// ==============================
// Implementation
// ==============================

template <typename T>
Plane<T>::Plane() : m_coefficients{0, 0, 0, 0} {}

template <typename T>
Plane<T>::Plane(T a, T b, T c, T d) : m_coefficients{a, b, c, d} {}

template <typename T>
Plane<T>::Plane(const Point<T, 3>& A,
                const Point<T, 3>& B,
                const Point<T, 3>& C) {
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
    for (auto& c : m_coefficients)
        c = -c;
}

template <typename T>
void Plane<T>::Normalize() {
    T len = std::sqrt(m_coefficients[0] * m_coefficients[0] +
                      m_coefficients[1] * m_coefficients[1] +
                      m_coefficients[2] * m_coefficients[2]);

    if (len > std::numeric_limits<T>::epsilon()) {
        for (auto& c : m_coefficients)
            c /= len;
    }
}

template <typename T>
void Plane<T>::Print() const {
    std::cout << "Plane coefficients: " << m_coefficients[0] << " "
              << m_coefficients[1] << " " << m_coefficients[2] << " "
              << m_coefficients[3] << "\n";
}

template <typename T>
Vector<T, 3> Plane<T>::GetNormalVector() const {
    return Vector<T, 3>{
        m_coefficients[0], m_coefficients[1], m_coefficients[2]};
}

template <typename T>
std::optional<T> Plane<T>::GetIntersection(const Ray<T, 3>& ray) const {
    const Vector<T, 3> rayDirection = ray.GetDirection();

    const T denominator = m_coefficients[0] * rayDirection[0] +
                          m_coefficients[1] * rayDirection[1] +
                          m_coefficients[2] * rayDirection[2];

    if (std::abs(denominator) < std::numeric_limits<T>::epsilon())
        return std::nullopt;  // Ray is parallel to the plane

    const Point<T, 3> rayOriginPoint = ray.GetOrigin();
    const T numerator =
        -(m_coefficients[0] * rayOriginPoint[0] +
          m_coefficients[1] * rayOriginPoint[1] +
          m_coefficients[2] * rayOriginPoint[2] + m_coefficients[3]);

    const T t = numerator / denominator;

    if (t < static_cast<T>(0))
        return std::nullopt;  // Intersection is behind the ray origin

    return t;
}

template <typename T>
void Plane<T>::Reflect(Ray<T, 3>& ray, T t) const {
    const Point<T, 3> intersectionPoint = ray.GetPosition(t);

    Vector<T, 3> normal = GetNormalVector();
    normal.NormalizeInPlace();

    ray.Normalize();
    const Vector<T, 3> incomingDirection = ray.GetDirection();

    Vector<T, 3> reflectedDirection =
        incomingDirection -
        normal * (static_cast<T>(2) * DotProduct(incomingDirection, normal));
    reflectedDirection.NormalizeInPlace();

    ray.Set(intersectionPoint, reflectedDirection);
}

// ==============================
// Non-member functions
// ==============================

/**
 * @brief Computes the intersection line of two planes as a Ray.
 *
 * Both planes must be normalized before calling this function.
 *
 * @return A Ray whose origin lies on the intersection line and whose
 *         direction is along it, or std::nullopt if the planes are parallel.
 */
template <typename T>
std::optional<Ray<T, 3>> IntersectPlanes(const Plane<T>& p1,
                                         const Plane<T>& p2) {
    const Vector<T, 3> n1 = p1.GetNormalVector();
    const Vector<T, 3> n2 = p2.GetNormalVector();
    const T d1 = p1.GetCoefficients()[3];
    const T d2 = p2.GetCoefficients()[3];

    // Direction of the intersection line: n1 x n2
    const Vector<T, 3> dir = CrossProduct(n1, n2);

    if (dir.MagnitudeSquared() < std::numeric_limits<T>::epsilon())
        return std::nullopt;  // Planes are parallel or coincident

    // Find a point on the intersection line by zeroing the coordinate
    // corresponding to the largest component of dir (most numerically stable).
    const T ax = std::abs(dir[0]);
    const T ay = std::abs(dir[1]);
    const T az = std::abs(dir[2]);
    const int k = (ax >= ay && ax >= az) ? 0 : (ay >= az) ? 1 : 2;

    // Indices of the two coordinates we solve for (the third is set to 0)
    const int i = (k + 1) % 3;
    const int j = (k + 2) % 3;

    // Solve the 2x2 system: n1[i]*pi + n1[j]*pj = -d1
    //                        n2[i]*pi + n2[j]*pj = -d2
    const T det = n1[i] * n2[j] - n1[j] * n2[i];

    // det cannot be near-zero here given dir is non-degenerate, but guard
    // anyway
    if (std::abs(det) < std::numeric_limits<T>::epsilon())
        return std::nullopt;

    T coords[3] = {0, 0, 0};
    coords[i] = ((-d1) * n2[j] - (-d2) * n1[j]) / det;
    coords[j] = (n1[i] * (-d2) - n2[i] * (-d1)) / det;

    const Point<T, 3> p0{coords[0], coords[1], coords[2]};

    return Ray<T, 3>(p0, dir.Normalize());
}

// ==============================
// Convenience type aliases
// ==============================

using Planef = Plane<float>;

}  // namespace RTB

#endif /* PLANE_HPP */