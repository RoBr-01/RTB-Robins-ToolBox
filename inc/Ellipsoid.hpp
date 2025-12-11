#ifndef ELLIPSOID_HPP
#define ELLIPSOID_HPP

// STL
#include <array>
// #include <source_location>
#include <cmath>
#include <iostream>

// RTB
#include "Math.hpp"
#include "Plane.hpp"
#include "Ray.hpp"

namespace RTB {

template <typename T>
struct trace_results {
    std::array<T, 2> pathlengths;
    std::array<T, 2> arclengths;
    std::array<Point<T, 3>, 2> tangent_points;
    std::array<Vector<T, 3>, 2> direct_rays;  // source to tangent
};

template <typename T>
struct EllipseParams {
    Point<T, 3> center;
    Vector<T, 3> semi_axes[2];  // Two semi-axis vectors
    T semi_axis_lengths[2];     // Lengths of semi-axes
    Vector<T, 3> normal;        // Plane normal
};

template <typename T>
class Ellipsoid {
   public:
    Ellipsoid();
    Ellipsoid(T length, T width, T height);
    Ellipsoid(std::initializer_list<T> sizes);
    Ellipsoid(std::array<T, 3> sizes);
    ~Ellipsoid() = default;
    const std::array<T, 3>& GetDimensions() const;

    std::array<T, 2> Intersect(const Ray<T, 3>& ray) const;
    EllipseParams<T> Intersect(const Plane<T>& plane) const;
    trace_results<T> TracePath(const Point<T, 3>& Source,
                               const std::array<Point<T, 3>, 2>& ears) const;

   private:
    T ArcLength(const Point<T, 3>& p1,
                const Point<T, 3>& p2,
                const Plane<T>& plane) const;

   private:
    std::array<T, 3> m_dimensions;
    Point<T, 3> m_origin = {0, 0, 0};
    // Quaternion<T> m_rotation{};
};

template <typename T>
Ellipsoid<T>::Ellipsoid() {
    m_dimensions = {0, 0, 0};
};

template <typename T>
Ellipsoid<T>::Ellipsoid(T length, T width, T height) {
    m_dimensions = {length, width, height};
};

template <typename T>
Ellipsoid<T>::Ellipsoid(std::initializer_list<T> sizes) {
    std::copy(sizes.begin(), sizes.end(), m_dimensions.begin());
};

template <typename T>
Ellipsoid<T>::Ellipsoid(std::array<T, 3> sizes) : m_dimensions{sizes} {};

template <typename T>
const std::array<T, 3>& Ellipsoid<T>::GetDimensions() const {
    return m_dimensions;
};

template <typename T>
std::array<T, 2> Ellipsoid<T>::Intersect(const Ray<T, 3>& ray) const {
    auto direction = ray.GetDirection();
    auto origin = ray.GetOrigin();
    T vx = direction[0], vy = direction[1], vz = direction[2];
    T x0 = origin[0], y0 = origin[1], z0 = origin[2];
    T A = m_dimensions[0], B = m_dimensions[1], C = m_dimensions[2];
    // Correct coefficient calculation
    T a = (vx * vx) / (A * A) + (vy * vy) / (B * B) + (vz * vz) / (C * C);
    T b = 2 * ((x0 * vx) / (A * A) + (y0 * vy) / (B * B) + (z0 * vz) / (C * C));
    T c = (x0 * x0) / (A * A) + (y0 * y0) / (B * B) + (z0 * z0) / (C * C) - 1;
    T d = b * b - 4 * a * c;
    if (d < 0) {
        return {};  // No intersection
    }
    if (d == 0) {
        return {-b / (2 * a), -1};  // Tangent
    }
    T sqrt_d = std::sqrt(d);
    T t1 = (-b + sqrt_d) / (2 * a);
    T t2 = (-b - sqrt_d) / (2 * a);
    return {t1, t2};
}

template <typename T>
EllipseParams<T> Ellipsoid<T>::Intersect(const Plane<T>& plane) const {
    EllipseParams<T> result;
    T A = m_dimensions[0], B = m_dimensions[1], C = m_dimensions[2];
    T A_sq = A * A, B_sq = B * B, C_sq = C * C;
    // Get plane coefficients from std::array
    T a = plane.GetCoefficients()[0];
    T b = plane.GetCoefficients()[1];
    T c = plane.GetCoefficients()[2];
    T d = plane.GetCoefficients()[3];
    // Center of intersection ellipse using initializer list
    result.center = Point<T, 3>{-a * d / (a * a + b * b + c * c),
                                -b * d / (a * a + b * b + c * c),
                                -c * d / (a * a + b * b + c * c)};
    result.normal = plane.GetNormalVector();
    result.normal.NormalizeInPlace();
    // Create orthonormal basis in the plane
    Vector<T, 3> u, v;
    if (std::abs(a) < 0.9) {
        u = CrossProduct(Vector<T, 3>{1, 0, 0}, result.normal);
    } else {
        u = CrossProduct(Vector<T, 3>{0, 1, 0}, result.normal);
    }
    u.NormalizeInPlace();
    v = CrossProduct(result.normal, u);
    // Matrix representation of ellipsoid: diag(1/A², 1/B², 1/C²)
    // Project onto plane to get 2D quadratic form
    T Q_uu = u[0] * u[0] / A_sq + u[1] * u[1] / B_sq + u[2] * u[2] / C_sq;
    T Q_vv = v[0] * v[0] / A_sq + v[1] * v[1] / B_sq + v[2] * v[2] / C_sq;
    T Q_uv = u[0] * v[0] / A_sq + u[1] * v[1] / B_sq + u[2] * v[2] / C_sq;
    // Solve eigenvalue problem for semi-axes
    T trace = Q_uu + Q_vv;
    T det = Q_uu * Q_vv - Q_uv * Q_uv;
    T discriminant = std::sqrt(trace * trace - 4 * det);
    T lambda1 = (trace + discriminant) / 2;
    T lambda2 = (trace - discriminant) / 2;
    result.semi_axis_lengths[0] = 1.0 / std::sqrt(lambda1);
    result.semi_axis_lengths[1] = 1.0 / std::sqrt(lambda2);
    // Eigenvectors give semi-axis directions
    T angle = 0.5 * std::atan2(2 * Q_uv, Q_uu - Q_vv);
    result.semi_axes[0] = std::cos(angle) * u + std::sin(angle) * v;
    result.semi_axes[1] = -std::sin(angle) * u + std::cos(angle) * v;
    return result;
}

template <typename T>
trace_results<T> Ellipsoid<T>::TracePath(
    const Point<T, 3>& source, const std::array<Point<T, 3>, 2>& ears) const {
    trace_results<T> results{};
    // Split for ease of use
    T A = m_dimensions[0];
    T B = m_dimensions[1];
    T C = m_dimensions[2];

    T A_sq = A * A;
    T B_sq = B * B;
    T C_sq = C * C;

    T Sx = source[0];
    T Sy = source[1];
    T Sz = source[2];

    // Construct the polar plane from the source point and ellipsoid
    T polar_a = Sx / A_sq;
    T polar_b = Sy / B_sq;
    T polar_c = Sz / C_sq;
    Plane<T> polar_plane(polar_a, polar_b, polar_c, -1);

    polar_plane.Normalize();

    for (size_t Currentear = 0; Currentear < 2; Currentear++) {
        const Point<T, 3> ear = ears[Currentear];

        // First check if the straight path to the ear crosses the Ellipsoid, if
        // it doesn't then we're done
        Vector<T, 3> to_ear(source, ear);
        Ray<T, 3> source_to_ear(source, to_ear);
        std::array<T, 2> straight_intersections = Intersect(source_to_ear);
        bool path_clear = true;
        const T epsilon = std::sqrt(std::numeric_limits<T>::epsilon());
        for (T t : straight_intersections) {
            // Check if intersection is within the segment
            if (t > epsilon && t < 1 - epsilon) {
                path_clear = false;
                break;
            }
        }
        T ear_distance = to_ear.Magnitude();
        if (path_clear) {
            // Straight path doesn't cross ellipsoid
            results.pathlengths[Currentear] = ear_distance;
            results.arclengths[Currentear] = 0;
            results.tangent_points[Currentear] = ear;
            results.direct_rays[Currentear] = to_ear;

            continue;
        }

        Plane<T> STO_plane(m_origin, ear, source);
        STO_plane.Normalize();

        Ray<T, 3> intersection_line = IntersectPlanes(polar_plane, STO_plane);

        std::array<T, 2> intersections = Intersect(intersection_line);

        Point<T, 3> tangent_one =
            intersection_line.GetPosition(intersections[0]);
        Point<T, 3> tangent_two =
            intersection_line.GetPosition(intersections[1]);

        // Now determine what tangent point is closer to this ear
        T length_one = distance_2points(ear, tangent_one);
        T length_two = distance_2points(ear, tangent_two);

        if (length_one < length_two - epsilon) {
            results.tangent_points[Currentear] = tangent_one;
        } else {
            results.tangent_points[Currentear] = tangent_two;
        }

        // Direct ray
        Vector<T, 3> direct_ray(source, results.tangent_points[Currentear]);
        results.direct_rays[Currentear] = direct_ray;

        // Direct length (since vector hasnt been normalised yet)
        T direct_length = direct_ray.Magnitude();

        // Correct until here!
        // Arc length
        T arc_length =
            ArcLength(results.tangent_points[Currentear], ear, STO_plane);
        results.pathlengths[Currentear] = direct_length + arc_length;
        results.arclengths[Currentear] = arc_length;
    }

    return results;
}

template <typename T>
struct LocalCoords {
    T u, v;   // Coordinates in ellipse's local 2D space
    T theta;  // Parametric angle
};

template <typename T>
LocalCoords<T> TransformToLocalCoords(const Point<T, 3>& point,
                                      const EllipseParams<T>& ellipse) {
    LocalCoords<T> result;

    // Translate to ellipse center using initializer list constructor
    Point<T, 3> translated_point = {point[0] - ellipse.center[0],
                                    point[1] - ellipse.center[1],
                                    point[2] - ellipse.center[2]};
    Vector<T, 3> translated(translated_point);

    // Project onto ellipse's local basis vectors using DotProduct
    result.u = DotProduct(translated, ellipse.semi_axes[0]) /
               ellipse.semi_axis_lengths[0];
    result.v = DotProduct(translated, ellipse.semi_axes[1]) /
               ellipse.semi_axis_lengths[1];

    // Convert to parametric angle
    result.theta = std::atan2(result.v, result.u);
    if (result.theta < 0)
        result.theta += 2 * M_PI;

    return result;
}

template <typename T>
T Ellipsoid<T>::ArcLength(const Point<T, 3>& p1,
                          const Point<T, 3>& p2,
                          const Plane<T>& plane) const {
    // Get ellipse parameters from plane-ellipsoid intersection
    EllipseParams<T> ellipse = Intersect(plane);

    // Transform both points to local coordinates
    LocalCoords<T> local1 = TransformToLocalCoords(p1, ellipse);
    LocalCoords<T> local2 = TransformToLocalCoords(p2, ellipse);

    // Get semi-axis lengths
    T a = ellipse.semi_axis_lengths[0];
    T b = ellipse.semi_axis_lengths[1];

    // Parametric angles
    T t1 = local1.theta;
    T t2 = local2.theta;

    // Handle angle wrapping for shortest path
    T angle_diff = t2 - t1;
    if (angle_diff < 0)
        angle_diff += 2 * M_PI;
    if (angle_diff > M_PI) {
        angle_diff = 2 * M_PI - angle_diff;
        std::swap(t1, t2);
    }

    // Polynomial approximation with Newton correction
    T ratio = b / a;
    T ratio_sq = ratio * ratio;
    T ratio_cu = ratio_sq * ratio;

    // 4th degree polynomial approximation
    T poly_approx = angle_diff * a *
                    (1.0 + 0.25 * ratio_sq - 0.125 * ratio_cu +
                     0.0625 * ratio_sq * ratio_sq);

    // Newton-Raphson correction (1-2 iterations)
    T corrected = poly_approx;
    for (int iter = 0; iter < 2; ++iter) {
        T sin_t = std::sin(corrected / a);
        T cos_t = std::cos(corrected / a);
        T f = std::sqrt(a * a * sin_t * sin_t + b * b * cos_t * cos_t) -
              std::sqrt(a * a * sin_t * sin_t + b * b * cos_t * cos_t);
        T df = (b * b - a * a) * sin_t * cos_t /
               std::sqrt(a * a * sin_t * sin_t + b * b * cos_t * cos_t);

        if (std::abs(df) > 1e-10) {
            corrected -= f / df;
        }
    }

    return corrected;
}

}  // namespace RTB

#endif /* ELLIPSOID_HPP */
