#ifndef PLANE_HPP
#define PLANE_HPP

// STL
#include <array>
#include <cmath>
#include <cstddef>
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
 * Coefficients are stored as {a, b, c, d}. Call normalize() to ensure
 * the normal vector (a, b, c) is unit length, which is required by
 * getIntersection(), reflect(), and IntersectPlanes().
 */
template <typename T>
class Plane {
   public:
    Plane();
    Plane(T coeff_a, T coeff_b, T coeff_c, T coeff_d);
    Plane(const Point<T, 3>& point_A,
          const Point<T, 3>& point_B,
          const Point<T, 3>& point_C);

    /**
     * @brief Flips the plane normal (negates all coefficients).
     */
    void invert();

    /**
     * @brief Normalizes the plane so that (a, b, c) is a unit vector.
     *
     * Also scales d accordingly. Required before using getIntersection(),
     * reflect(), or IntersectPlanes().
     */
    void normalize();

    void print() const;

    /**
     * @brief Returns the normal vector (a, b, c). Not guaranteed unit length
     *        unless normalize() has been called.
     */
    [[nodiscard]] Vector<T, 3> getNormalVector() const;

    /**
     * @brief Returns the ray parameter t at which the ray intersects the plane.
     *
     * Returns std::nullopt if the ray is parallel to the plane or the
     * intersection is behind the ray origin (t < 0).
     *
     * Assumes the plane is normalized.
     */
    [[nodiscard]] std::optional<T> getIntersection(const Ray<T, 3>& ray) const;

    /**
     * @brief Reflects the ray about the plane at parameter t.
     *
     * Assumes the plane is normalized.
     */
    void reflect(Ray<T, 3>& ray, T step) const;

    [[nodiscard]] std::array<T, 4> getCoefficients() const {
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
Plane<T>::Plane(T coeff_a, T coeff_b, T coeff_c, T coeff_d)
    : m_coefficients{coeff_a, coeff_b, coeff_c, coeff_d} {}

template <typename T>
Plane<T>::Plane(const Point<T, 3>& point_A,
                const Point<T, 3>& point_B,
                const Point<T, 3>& point_C) {
    const T point_a_0 = point_A[0];
    const T point_a_1 = point_A[1];
    const T point_a_2 = point_A[2];

    const T point_b_0 = point_B[0];
    const T point_b_1 = point_B[1];
    const T point_b_2 = point_B[2];

    const T point_c_0 = point_C[0];
    const T point_c_1 = point_C[1];
    const T point_c_2 = point_C[2];

    m_coefficients[0] = (point_b_1 - point_a_1) * (point_c_2 - point_a_2) -
                        (point_c_1 - point_a_1) * (point_b_2 - point_a_2);
    m_coefficients[1] = (point_b_2 - point_a_2) * (point_c_0 - point_a_0) -
                        (point_c_2 - point_a_2) * (point_b_0 - point_a_0);
    m_coefficients[2] = (point_b_0 - point_a_0) * (point_c_1 - point_a_1) -
                        (point_c_0 - point_a_0) * (point_b_1 - point_a_1);
    m_coefficients[3] =
        -(m_coefficients[0] * point_a_0 + m_coefficients[1] * point_a_1 +
          m_coefficients[2] * point_a_2);
}

template <typename T>
void Plane<T>::invert() {
    for (auto& coeff : m_coefficients) {
        coeff = -coeff;
    }
}

template <typename T>
void Plane<T>::normalize() {
    T len = std::sqrt(m_coefficients[0] * m_coefficients[0] +
                      m_coefficients[1] * m_coefficients[1] +
                      m_coefficients[2] * m_coefficients[2]);

    if (len > std::numeric_limits<T>::epsilon()) {
        for (auto& coeff : m_coefficients) {
            coeff /= len;
        }
    }
}

template <typename T>
void Plane<T>::print() const {
    std::cout << "Plane coefficients: " << m_coefficients[0] << " "
              << m_coefficients[1] << " " << m_coefficients[2] << " "
              << m_coefficients[3] << "\n";
}

template <typename T>
Vector<T, 3> Plane<T>::getNormalVector() const {
    return Vector<T, 3>{
        m_coefficients[0], m_coefficients[1], m_coefficients[2]};
}

template <typename T>
std::optional<T> Plane<T>::getIntersection(const Ray<T, 3>& ray) const {
    const Vector<T, 3> raydirection = ray.getDirection();

    const T denominator = m_coefficients[0] * raydirection[0] +
                          m_coefficients[1] * raydirection[1] +
                          m_coefficients[2] * raydirection[2];

    if (std::abs(denominator) < std::numeric_limits<T>::epsilon()) {
        return std::nullopt;  // Ray is parallel to the plane
    }

    const Point<T, 3> rayorigin = ray.getOrigin();
    const T numerator =
        -(m_coefficients[0] * rayorigin[0] + m_coefficients[1] * rayorigin[1] +
          m_coefficients[2] * rayorigin[2] + m_coefficients[3]);

    const T step = numerator / denominator;

    if (step < static_cast<T>(0)) {
        return std::nullopt;  // Intersection is behind the ray origin
    }

    return step;
}

template <typename T>
void Plane<T>::reflect(Ray<T, 3>& ray, T step) const {
    const Point<T, 3> intersectionpoint = ray.getPosition(step);

    Vector<T, 3> normal = getNormalVector();
    normal.normalizeInPlace();

    ray.normalize();
    const Vector<T, 3> incomingdirection = ray.getDirection();

    Vector<T, 3> reflecteddirection =
        incomingdirection -
        normal * (static_cast<T>(2) * dotProduct(incomingdirection, normal));
    reflecteddirection.normalizeInPlace();

    ray.set(intersectionpoint, reflecteddirection);
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
std::optional<Ray<T, 3>> intersectPlanes(const Plane<T>& plane_1,
                                         const Plane<T>& plane_2) {
    const Vector<T, 3> normal_1 = plane_1.getNormalVector();
    const Vector<T, 3> normal_2 = plane_2.getNormalVector();
    const T coeff_d_1 = plane_1.getCoefficients()[3];
    const T coeff_d_2 = plane_2.getCoefficients()[3];

    // Direction of the intersection line: normal_1 x normal_2
    const Vector<T, 3> dir = crossProduct(normal_1, normal_2);

    if (dir.magnitudeSquared() < std::numeric_limits<T>::epsilon()) {
        return std::nullopt;  // Planes are parallel or coincident
    }

    // Find a point on the intersection line by zeroing the coordinate
    // corresponding to the largest component of dir (most numerically stable).
    const T directionx = std::abs(dir[0]);
    const T directiony = std::abs(dir[1]);
    const T directionz = std::abs(dir[2]);
    size_t index_a;

    if (directionx >= directiony && directionx >= directionz) {
        index_a = 0;
    } else if (directiony >= directionz) {
        index_a = 1;
    } else {
        index_a = 2;
    }

    // Indices of the two coordinates we solve for (the third is set to 0)
    const size_t index_b = (index_a + 1) % 3;
    const size_t index_c = (index_a + 2) % 3;

    // Solve the 2x2 system: normal_1[i]*pi + normal_1[j]*pj = -coeff_d_1
    //                        normal_2[i]*pi + normal_2[j]*pj = -coeff_d_2
    const T det = normal_1[index_b] * normal_2[index_c] -
                  normal_1[index_c] * normal_2[index_b];

    // det cannot be near-zero here given dir is non-degenerate, but guard
    // anyway
    if (std::abs(det) < std::numeric_limits<T>::epsilon()) {
        return std::nullopt;
    }

    std::array<T, 3> coords{};
    coords[index_b] =
        ((-coeff_d_1) * normal_2[index_c] - (-coeff_d_2) * normal_1[index_c]) /
        det;
    coords[index_c] =
        (normal_1[index_b] * (-coeff_d_2) - normal_2[index_b] * (-coeff_d_1)) /
        det;

    const Point<T, 3> point0{coords[0], coords[1], coords[2]};

    return Ray<T, 3>(point0, dir.normalize());
}

}  // namespace RTB

#endif /* PLANE_HPP */