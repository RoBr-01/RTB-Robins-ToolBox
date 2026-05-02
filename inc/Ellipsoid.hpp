#ifndef ELLIPSOID_HPP
#define ELLIPSOID_HPP

// STL
#include <array>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <optional>

// RTB
#include "Math.hpp"
#include "Plane.hpp"
#include "Ray.hpp"

namespace RTB {

// ==============================
// ArcLength method policy
// ==============================

/**
 * @brief Compile-time selection of arc length computation method.
 *
 * - Polynomial:          O(1). Fast, reasonable for near-circular ellipses
 *                        (b/a > ~0.8). Degrades for high eccentricity.
 * - Ramanujan:           O(1). ~0.0000003% error for full perimeters.
 *                        Degrades for partial arcs of eccentric ellipses
 *                        due to uniform-distribution assumption.
 * - GaussianQuadrature:  O(n), n=10 points. Effectively full floating-point
 *                        precision for any eccentricity and arc position.
 *                        Default.
 */
enum class ArcLengthMethod { Polynomial, Ramanujan, GaussianQuadrature };

// ==============================
// Supporting types
// ==============================

/**
 * @brief A single diffraction path from source to one ear.
 */
template <typename T>
struct EarPath {
    T pathlength;
    T arclength;
    Point<T, 3> tangent_point;
    Vector<T, 3> direct_ray;  // source to tangent point (unnormalised)
};

/**
 * @brief Results of TracePath for both ears.
 *
 * Each ear has two paths: [0] is the shorter (primary) path,
 * [1] is the longer (secondary) path. For an unoccluded direct path,
 * both entries carry the same direct ray with arclength = 0.
 */
template <typename T>
struct TraceResults {
    std::array<EarPath<T>, 2> left_ear_paths;   // [0] shorter, [1] longer
    std::array<EarPath<T>, 2> right_ear_paths;  // [0] shorter, [1] longer
};

template <typename T>
struct EllipseParams {
    Point<T, 3> center;
    std::array<Vector<T, 3>, 2> semi_axes;  // unit semi-axis vectors
    std::array<T, 2> semi_axis_lengths;     // lengths of semi-axes
    Vector<T, 3> normal;                    // plane normal (unit)
};

template <typename T>
struct LocalCoords {
    T u, v;   // coordinates in ellipse's local 2D space
    T theta;  // parametric angle [0, 2*PI)
};

// ==============================
// Ellipsoid
// ==============================

/**
 * @brief Axis-aligned ellipsoid centred at the origin.
 *
 * @tparam T      Scalar type.
 * @tparam Method Arc length computation method. Defaults to Ramanujan.
 */
template <typename T,
          ArcLengthMethod Method = ArcLengthMethod::GaussianQuadrature>
class Ellipsoid {
   public:
    Ellipsoid();
    Ellipsoid(T length, T width, T height);
    explicit Ellipsoid(std::initializer_list<T> sizes);
    explicit Ellipsoid(const std::array<T, 3>& sizes);

    const std::array<T, 3>& GetDimensions() const;

    /**
     * @brief Ray-ellipsoid intersection.
     *
     * Returns the two ray parameters t1, t2 at which the ray intersects,
     * or std::nullopt if there is no intersection.
     * For a tangent intersection t1 == t2.
     */
    std::optional<std::array<T, 2>> IntersectRay(const Ray<T, 3>& ray) const;

    /**
     * @brief Plane-ellipsoid intersection.
     *
     * Returns the ellipse formed by the intersection.
     * Assumes the plane is normalized.
     */
    EllipseParams<T> IntersectPlane(const Plane<T>& plane) const;

    /**
     * @brief Traces the diffraction path from a source point to two ear points
     *        around the ellipsoid surface.
     */
    TraceResults<T> TracePath(const Point<T, 3>& source,
                              const std::array<Point<T, 3>, 2>& ears) const;

   private:
    /**
     * @brief Returns the arc length along the ellipse cross-section between
     *        p1 and p2, using the compile-time selected Method.
     */
    T ArcLength(const Point<T, 3>& p1,
                const Point<T, 3>& p2,
                const Plane<T>& plane) const;

    std::array<T, 3> m_dimensions;
    Point<T, 3> m_origin = {0, 0, 0};
};

// ==============================
// Implementation
// ==============================

template <typename T, ArcLengthMethod Method>
Ellipsoid<T, Method>::Ellipsoid() : m_dimensions{0, 0, 0} {}

template <typename T, ArcLengthMethod Method>
Ellipsoid<T, Method>::Ellipsoid(T length, T width, T height)
    : m_dimensions{length, width, height} {}

template <typename T, ArcLengthMethod Method>
Ellipsoid<T, Method>::Ellipsoid(std::initializer_list<T> sizes) {
    assert(sizes.size() == 3);
    std::copy(sizes.begin(), sizes.end(), m_dimensions.begin());
}

template <typename T, ArcLengthMethod Method>
Ellipsoid<T, Method>::Ellipsoid(const std::array<T, 3>& sizes)
    : m_dimensions(sizes) {}

template <typename T, ArcLengthMethod Method>
const std::array<T, 3>& Ellipsoid<T, Method>::GetDimensions() const {
    return m_dimensions;
}

template <typename T, ArcLengthMethod Method>
std::optional<std::array<T, 2>> Ellipsoid<T, Method>::IntersectRay(
    const Ray<T, 3>& ray) const {
    const Vector<T, 3> dir = ray.GetDirection();
    const Point<T, 3> origin = ray.GetOrigin();

    const T vx = dir[0], vy = dir[1], vz = dir[2];
    const T x0 = origin[0], y0 = origin[1], z0 = origin[2];
    const T A = m_dimensions[0], B = m_dimensions[1], C = m_dimensions[2];

    const T a = (vx * vx) / (A * A) + (vy * vy) / (B * B) + (vz * vz) / (C * C);
    const T b = static_cast<T>(2) * ((x0 * vx) / (A * A) + (y0 * vy) / (B * B) +
                                     (z0 * vz) / (C * C));
    const T c = (x0 * x0) / (A * A) + (y0 * y0) / (B * B) +
                (z0 * z0) / (C * C) - static_cast<T>(1);

    const T discriminant = b * b - static_cast<T>(4) * a * c;

    if (discriminant < static_cast<T>(0))
        return std::nullopt;

    const T sqrt_d = std::sqrt(discriminant);
    const T t1 = (-b + sqrt_d) / (static_cast<T>(2) * a);
    const T t2 = (-b - sqrt_d) / (static_cast<T>(2) * a);
    return std::array<T, 2>{t1, t2};
}

template <typename T, ArcLengthMethod Method>
EllipseParams<T> Ellipsoid<T, Method>::IntersectPlane(
    const Plane<T>& plane) const {
    EllipseParams<T> result;

    const T A = m_dimensions[0], B = m_dimensions[1], C = m_dimensions[2];
    const T A_sq = A * A, B_sq = B * B, C_sq = C * C;

    const auto coeffs = plane.GetCoefficients();
    const T a = coeffs[0], b = coeffs[1], c = coeffs[2], d = coeffs[3];
    const T n_sq = a * a + b * b + c * c;

    result.center = Point<T, 3>{-a * d / n_sq, -b * d / n_sq, -c * d / n_sq};
    result.normal = plane.GetNormalVector();
    result.normal.NormalizeInPlace();

    // Build orthonormal basis in the plane
    Vector<T, 3> u, v;
    if (std::abs(a) < static_cast<T>(0.9))
        u = CrossProduct(Vector<T, 3>{1, 0, 0}, result.normal);
    else
        u = CrossProduct(Vector<T, 3>{0, 1, 0}, result.normal);
    u.NormalizeInPlace();
    v = CrossProduct(result.normal, u);

    // Project ellipsoid metric onto the plane basis to get 2D quadratic form
    const T Q_uu = u[0] * u[0] / A_sq + u[1] * u[1] / B_sq + u[2] * u[2] / C_sq;
    const T Q_vv = v[0] * v[0] / A_sq + v[1] * v[1] / B_sq + v[2] * v[2] / C_sq;
    const T Q_uv = u[0] * v[0] / A_sq + u[1] * v[1] / B_sq + u[2] * v[2] / C_sq;

    // Eigendecomposition of the 2x2 quadratic form
    const T trace = Q_uu + Q_vv;
    const T det = Q_uu * Q_vv - Q_uv * Q_uv;
    const T discriminant = std::sqrt(trace * trace - static_cast<T>(4) * det);
    const T lambda1 = (trace + discriminant) / static_cast<T>(2);
    const T lambda2 = (trace - discriminant) / static_cast<T>(2);

    result.semi_axis_lengths[0] = static_cast<T>(1) / std::sqrt(lambda1);
    result.semi_axis_lengths[1] = static_cast<T>(1) / std::sqrt(lambda2);

    const T angle =
        static_cast<T>(0.5) * std::atan2(static_cast<T>(2) * Q_uv, Q_uu - Q_vv);
    result.semi_axes[0] = std::cos(angle) * u + std::sin(angle) * v;
    result.semi_axes[1] = -std::sin(angle) * u + std::cos(angle) * v;

    return result;
}

// ==============================
// Arc length helper (non-member)
// ==============================

template <typename T>
LocalCoords<T> TransformToLocalCoords(const Point<T, 3>& point,
                                      const EllipseParams<T>& ellipse) {
    LocalCoords<T> result;

    const Vector<T, 3> translated{point[0] - ellipse.center[0],
                                  point[1] - ellipse.center[1],
                                  point[2] - ellipse.center[2]};

    result.u = DotProduct(translated, ellipse.semi_axes[0]) /
               ellipse.semi_axis_lengths[0];
    result.v = DotProduct(translated, ellipse.semi_axes[1]) /
               ellipse.semi_axis_lengths[1];

    result.theta = std::atan2(result.v, result.u);
    if (result.theta < static_cast<T>(0))
        result.theta += static_cast<T>(2) * static_cast<T>(PI);

    return result;
}

template <typename T, ArcLengthMethod Method>
T Ellipsoid<T, Method>::ArcLength(const Point<T, 3>& p1,
                                  const Point<T, 3>& p2,
                                  const Plane<T>& plane) const {
    const EllipseParams<T> ellipse = IntersectPlane(plane);

    const LocalCoords<T> local1 = TransformToLocalCoords(p1, ellipse);
    const LocalCoords<T> local2 = TransformToLocalCoords(p2, ellipse);

    const T a = ellipse.semi_axis_lengths[0];
    const T b = ellipse.semi_axis_lengths[1];

    T t1 = local1.theta;
    T t2 = local2.theta;

    // Shortest arc
    T angle_diff = t2 - t1;
    if (angle_diff < static_cast<T>(0))
        angle_diff += static_cast<T>(2) * static_cast<T>(PI);
    if (angle_diff > static_cast<T>(PI)) {
        angle_diff = static_cast<T>(2) * static_cast<T>(PI) - angle_diff;
        std::swap(t1, t2);
    }

    if constexpr (Method == ArcLengthMethod::Polynomial) {
        // 4th-degree polynomial approximation.
        // Reasonable for near-circular ellipses (b/a > ~0.8),
        // degrades for high eccentricity.
        const T ratio = b / a;
        const T ratio_sq = ratio * ratio;
        const T ratio_cu = ratio_sq * ratio;
        const T ratio_qu = ratio_sq * ratio_sq;
        return angle_diff * a *
               (static_cast<T>(1) + static_cast<T>(0.25) * ratio_sq -
                static_cast<T>(0.125) * ratio_cu +
                static_cast<T>(0.0625) * ratio_qu);
    } else if constexpr (Method == ArcLengthMethod::Ramanujan) {
        // Ramanujan's second approximation for full ellipse perimeter,
        // scaled by angle_diff / (2*PI) for a partial arc.
        // Error ~0.0000003% for typical ellipses.
        const T h = ((a - b) * (a - b)) / ((a + b) * (a + b));
        const T perimeter =
            static_cast<T>(PI) * (a + b) *
            (static_cast<T>(1) +
             (static_cast<T>(3) * h) /
                 (static_cast<T>(10) +
                  std::sqrt(static_cast<T>(4) - static_cast<T>(3) * h)));
        return perimeter *
               (angle_diff / (static_cast<T>(2) * static_cast<T>(PI)));
    } else {
        // 10-point Gauss-Legendre quadrature.
        // Integrates sqrt(a²sin²t + b²cos²t) over [t1, t1+angle_diff].
        // Effectively full floating-point precision for any eccentricity.

        // Abscissae and weights for 10-point Gauss-Legendre on [-1, 1]
        static constexpr double gl_x[10] = {-0.9739065285,
                                            -0.8650633667,
                                            -0.6794095683,
                                            -0.4333953941,
                                            -0.1488743390,
                                            0.1488743390,
                                            0.4333953941,
                                            0.6794095683,
                                            0.8650633667,
                                            0.9739065285};
        static constexpr double gl_w[10] = {0.0666713443,
                                            0.1494513492,
                                            0.2190863625,
                                            0.2692667193,
                                            0.2955242247,
                                            0.2955242247,
                                            0.2692667193,
                                            0.2190863625,
                                            0.1494513492,
                                            0.0666713443};

        // Change of interval from [-1,1] to [t1, t1+angle_diff]
        const T half_range = angle_diff / static_cast<T>(2);
        const T mid = t1 + half_range;

        T integral = static_cast<T>(0);
        for (int i = 0; i < 10; ++i) {
            const T t = mid + half_range * static_cast<T>(gl_x[i]);
            const T sin_t = std::sin(t);
            const T cos_t = std::cos(t);
            const T integrand =
                std::sqrt(a * a * sin_t * sin_t + b * b * cos_t * cos_t);
            integral += static_cast<T>(gl_w[i]) * integrand;
        }

        return half_range * integral;
    }
}

// ==============================
// TracePath
// ==============================

template <typename T, ArcLengthMethod Method>
TraceResults<T> Ellipsoid<T, Method>::TracePath(
    const Point<T, 3>& source, const std::array<Point<T, 3>, 2>& ears) const {
    TraceResults<T> results{};

    const T A = m_dimensions[0];
    const T B = m_dimensions[1];
    const T C = m_dimensions[2];

    const T A_sq = A * A, B_sq = B * B, C_sq = C * C;
    const T Sx = source[0], Sy = source[1], Sz = source[2];

    const T epsilon = std::sqrt(std::numeric_limits<T>::epsilon());

    // Source must be strictly outside the ellipsoid
    assert(Sx * Sx / A_sq + Sy * Sy / B_sq + Sz * Sz / C_sq >
               static_cast<T>(1) + epsilon &&
           "Source must be strictly outside the ellipsoid.");

    // Polar plane of the source point w.r.t. the ellipsoid:
    // (Sx/A²)x + (Sy/B²)y + (Sz/C²)z = 1
    T polar_a = Sx / A_sq;
    T polar_b = Sy / B_sq;
    T polar_c = Sz / C_sq;

    // Scale for numerical stability with small dimensions
    const T scale_factor =
        std::max({std::abs(polar_a), std::abs(polar_b), std::abs(polar_c)});

    Plane<T> polar_plane;
    if (scale_factor > static_cast<T>(1000)) {
        polar_a /= scale_factor;
        polar_b /= scale_factor;
        polar_c /= scale_factor;
        polar_plane = Plane<T>(
            polar_a, polar_b, polar_c, static_cast<T>(-1) / scale_factor);
    } else {
        polar_plane = Plane<T>(polar_a, polar_b, polar_c, static_cast<T>(-1));
    }
    polar_plane.Normalize();

    // Projects a point radially onto the ellipsoid surface.
    // Corrects for floating-point drift — ears are expected to lie on the
    // surface, so we enforce it exactly before using them geometrically.
    // Note: radial projection from origin, not closest-point (quartic).
    // For points already near the surface the difference is negligible.
    auto ProjectOntoSurface = [&](const Point<T, 3>& p) -> Point<T, 3> {
        const T scale = std::sqrt(p[0] * p[0] / A_sq + p[1] * p[1] / B_sq +
                                  p[2] * p[2] / C_sq);
        return Point<T, 3>{p[0] / scale, p[1] / scale, p[2] / scale};
    };

    // Computes a single EarPath given a tangent point and ear.
    auto MakeEarPath = [&](const Point<T, 3>& tangent,
                           const Point<T, 3>& ear,
                           const Plane<T>& sto_plane) -> EarPath<T> {
        EarPath<T> path;
        path.tangent_point = tangent;
        path.direct_ray = Vector<T, 3>(source, tangent);
        const T direct_length = path.direct_ray.Magnitude();
        path.arclength = ArcLength(tangent, ear, sto_plane);
        path.pathlength = direct_length + path.arclength;
        return path;
    };

    for (std::size_t earIdx = 0; earIdx < 2; ++earIdx) {
        const Point<T, 3> ear = ProjectOntoSurface(ears[earIdx]);
        std::array<EarPath<T>, 2>& ear_paths =
            (earIdx == 0) ? results.left_ear_paths : results.right_ear_paths;

        // Check if the straight source-to-ear path is occluded.
        // to_ear is unnormalised — t=1 corresponds exactly to the ear position,
        // so the occlusion window is (epsilon, 1-epsilon) in ray-parameter
        // space.
        const Vector<T, 3> to_ear(source, ear);
        const Ray<T, 3> source_to_ear(source, to_ear);

        const auto straight_intersections = IntersectRay(source_to_ear);
        bool path_clear = true;
        if (straight_intersections.has_value()) {
            for (const T t : straight_intersections.value()) {
                if (t > epsilon && t < static_cast<T>(1) - epsilon) {
                    path_clear = false;
                    break;
                }
            }
        }

        if (path_clear) {
            // No occlusion — both path slots carry the direct path.
            EarPath<T> direct{};
            direct.pathlength = to_ear.Magnitude();
            direct.arclength = static_cast<T>(0);
            direct.tangent_point = ear;
            direct.direct_ray = to_ear;
            ear_paths[0] = direct;
            ear_paths[1] = direct;
            continue;
        }

        // Diffraction plane through origin, ear, and source
        Plane<T> STO_plane(m_origin, ear, source);
        STO_plane.Normalize();

        // Intersection of polar plane and diffraction plane gives the tangent
        // line
        const auto intersection_line = IntersectPlanes(polar_plane, STO_plane);
        assert(intersection_line.has_value() &&
               "Polar plane and diffraction plane are parallel — degenerate geometry.");

        const auto intersections = IntersectRay(intersection_line.value());
        assert(
            intersections.has_value() &&
            "Tangent line does not intersect ellipsoid — degenerate geometry.");

        const Point<T, 3> tangent_one =
            intersection_line->GetPosition(intersections.value()[0]);
        const Point<T, 3> tangent_two =
            intersection_line->GetPosition(intersections.value()[1]);

        // Compute both paths and sort: [0] shorter, [1] longer
        EarPath<T> path_one = MakeEarPath(tangent_one, ear, STO_plane);
        EarPath<T> path_two = MakeEarPath(tangent_two, ear, STO_plane);

        if (path_one.pathlength <= path_two.pathlength) {
            ear_paths[0] = path_one;
            ear_paths[1] = path_two;
        } else {
            ear_paths[0] = path_two;
            ear_paths[1] = path_one;
        }
    }

    return results;
}

// ==============================
// Convenience type aliases
// ==============================

using Ellipsoidf = Ellipsoid<float>;
using EllipsoiddRamanujan = Ellipsoid<double, ArcLengthMethod::Ramanujan>;

}  // namespace RTB

#endif /* ELLIPSOID_HPP */