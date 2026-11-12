/*
 ____ _____ ____
|  _ \_   _| __ )
| |_) || | |  _ \
|  _ < | | | |_) |
|_| \_\|_| |____/

 Ellipsoid.hpp
 3D ellipsoid: ray/plane intersection, path tracing, and arc length.

 Copyright (c) 2026 Robin Breugelmans
 Licensed under the MIT License. See LICENSE in the project root.
*/

#ifndef RTB_ELLIPSOID_HPP
#define RTB_ELLIPSOID_HPP

#include <RTB/Debug.hpp>
#include <RTB/Ellipse.hpp>
#include <RTB/Math.hpp>
#include <RTB/Plane.hpp>
#include <RTB/Point.hpp>
#include <RTB/Ray.hpp>
#include <RTB/Vector.hpp>
#include <array>
#include <cassert>
#include <cstddef>
#include <limits>
#include <optional>

namespace RTB {

/**
 * @brief Checks if three points are collinear within an epsilon tolerance.
 *
 * @tparam T Scalar type
 * @tparam N Dimension (2 or 3 typically)
 * @param epsilon Tolerance
 * @param p1 First point
 * @param p2 Second point
 * @param p3 Third point
 * @return true if points are collinear
 */
template <typename T, std::size_t N>
[[nodiscard]] bool areCollinear(T epsilon,
                                const Point<T, N>& p1,
                                const Point<T, N>& p2,
                                const Point<T, N>& p3) {
    // Vectors from p1 to p2 and p1 to p3
    Vector<T, N> v12;
    Vector<T, N> v13;

    for (std::size_t i = 0; i < N; ++i) {
        v12[i] = p2[i] - p1[i];
        v13[i] = p3[i] - p1[i];
    }

    // Compute squared cross product magnitude
    T cross_sq = T{0};
    for (std::size_t i = 0; i < N; ++i) {
        for (std::size_t j = i + 1; j < N; ++j) {
            T cross_comp = v12[i] * v13[j] - v12[j] * v13[i];
            cross_sq += cross_comp * cross_comp;
        }
    }

    // Compare against tolerance
    T mag_sq = T{0};
    for (std::size_t i = 0; i < N; ++i) {
        mag_sq += v12[i] * v12[i];
    }

    return cross_sq <= epsilon * epsilon * mag_sq;
}

/**
 * @brief A diffraction path from a source point around an ellipsoid to a
 * point on its surface.
 */
template <typename T>
struct Path {
    T total_length{};     ///< straight_length + arclength.
    T straight_length{};  ///< Length of source2tanpoint.
    T arclength{};        ///< Arc length along the ellipsoid's surface.
    Point<T, 3> tangent_point{};
    Vector<T, 3> source2tanpoint{};  ///< Source to tangent point, unnormalized.
};

template <typename T>
class Ellipsoid {
   public:
    Ellipsoid() = default;
    Ellipsoid(T length, T width, T height);

    [[nodiscard]] std::array<T, 3> getDimensions() const;

    /**
     * @brief Returns the ellipsoid's radius along the direction given by
     * azimuth/elevation (degrees).
     *
     * @warning Requires non-zero dimensions; a default-constructed
     * Ellipsoid (all dimensions zero) divides by zero here.
     */
    [[nodiscard]] T getRadiusAt(T a_azimuth, T a_elevation) const;

    void setDimensions(T a_depth, T a_width, T a_height);

    /**
     * @brief Returns the ray parameter(s) at which ray intersects the
     * ellipsoid: {t1, t2} for two intersections, {t, std::nullopt} for a
     * tangent ray, or {std::nullopt, std::nullopt} for no intersection.
     *
     * @warning Requires non-zero dimensions and a non-zero-length ray
     * direction; either divides by zero here.
     */
    [[nodiscard]] std::array<std::optional<T>, 2> intersectRay(
        const Ray<T, 3>& ray) const;

    [[nodiscard]] std::optional<Ellipse<T>> intersectPlane(
        const Plane<T>& plane) const;

    /**
     * @brief Returns the arc length of the ellipse formed by intersecting
     * the ellipsoid with the plane through point_1, point_2, and the
     * origin.
     *
     * Returns std::nullopt if that plane does not intersect the
     * ellipsoid in a well-defined ellipse (e.g. a degenerate/zero-size
     * ellipsoid).
     */
    [[nodiscard]] std::optional<T> arcLength(const Point<T, 3>& point_1,
                                             const Point<T, 3>& point_2) const;

    /**
     * @brief Traces the diffraction path(s) from point_outside around the
     * ellipsoid to point_on, a point on (or treated as on) its surface.
     *
     * Returns {path, std::nullopt} if point_on is directly visible from
     * point_outside (unobstructed by the ellipsoid) — only one physical
     * path exists in that case. Otherwise returns the two paths that
     * diffract around either side of the ellipsoid, {path_a, path_b}.
     * Returns {std::nullopt, std::nullopt} if no valid path could be
     * computed (e.g. a degenerate/zero-size ellipsoid).
     */
    [[nodiscard]] std::array<std::optional<Path<T>>, 2> tracePath(
        const Point<T, 3>& point_outside, const Point<T, 3>& point_on) const;

   private:
    std::array<T, 3> m_dimensions{0};
    Point<T, 3> m_origin;
    // Quaternion<T> m_rotation{};
};

template <typename T>
Ellipsoid<T>::Ellipsoid(T length, T width, T height)
    : m_dimensions{length, width, height} {}

template <typename T>
std::array<T, 3> Ellipsoid<T>::getDimensions() const {
    return m_dimensions;
}

template <typename T>
T Ellipsoid<T>::getRadiusAt(T a_azimuth, T a_elevation) const {
    const T dim_a = m_dimensions[0];
    const T dim_b = m_dimensions[1];
    const T dim_c = m_dimensions[2];

    assert(std::abs(dim_a) > std::numeric_limits<T>::epsilon() &&
           std::abs(dim_b) > std::numeric_limits<T>::epsilon() &&
           std::abs(dim_c) > std::numeric_limits<T>::epsilon() &&
           "Ellipsoid::getRadiusAt() requires non-zero dimensions.");

    const T cos_ele = RTB::cosd(a_elevation);
    const T sin_ele = RTB::sind(a_elevation);
    const T cos_azi = RTB::cosd(a_azimuth);
    const T sin_azi = RTB::sind(a_azimuth);

    // Direction (cos_ele*cos_azi, cos_ele*sin_azi, sin_ele) is a unit vector;
    // solving the ellipsoid equation (x/a)^2+(y/b)^2+(z/c)^2=1 for r along
    // that direction gives r = 1/sqrt(...).
    const T inv_radius_sq =
        (cos_ele * cos_ele * cos_azi * cos_azi) / (dim_a * dim_a) +
        (cos_ele * cos_ele * sin_azi * sin_azi) / (dim_b * dim_b) +
        (sin_ele * sin_ele) / (dim_c * dim_c);

    return static_cast<T>(1) / std::sqrt(inv_radius_sq);
}

template <typename T>
void Ellipsoid<T>::setDimensions(T a_depth, T a_width, T a_height) {
    m_dimensions = {a_depth, a_width, a_height};
}

template <typename T>
std::array<std::optional<T>, 2> Ellipsoid<T>::intersectRay(
    const Ray<T, 3>& ray) const {
    const auto dir = ray.getDirection();
    const Point<T, 3> origin = ray.getOrigin();

    const T vx = dir[0];
    const T vy = dir[1];
    const T vz = dir[2];
    const T x0 = origin[0];
    const T y0 = origin[1];
    const T z0 = origin[2];
    const T dim_a = m_dimensions[0];
    const T dim_b = m_dimensions[1];
    const T dim_c = m_dimensions[2];

    assert(std::abs(dim_a) > std::numeric_limits<T>::epsilon() &&
           std::abs(dim_b) > std::numeric_limits<T>::epsilon() &&
           std::abs(dim_c) > std::numeric_limits<T>::epsilon() &&
           "Ellipsoid::intersectRay() requires non-zero dimensions.");
    assert(dir.magnitudeSquared() > std::numeric_limits<T>::epsilon() &&
           "Ellipsoid::intersectRay() requires a non-zero-length ray "
           "direction.");

    const T coeff_a = (vx * vx) / (dim_a * dim_a) +
                      (vy * vy) / (dim_b * dim_b) + (vz * vz) / (dim_c * dim_c);
    const T coeff_b = static_cast<T>(2) * ((x0 * vx) / (dim_a * dim_a) +
                                           (y0 * vy) / (dim_b * dim_b) +
                                           (z0 * vz) / (dim_c * dim_c));
    const T coeff_c = (x0 * x0) / (dim_a * dim_a) +
                      (y0 * y0) / (dim_b * dim_b) +
                      (z0 * z0) / (dim_c * dim_c) - static_cast<T>(1);

    const T discriminant =
        coeff_b * coeff_b - static_cast<T>(4) * coeff_a * coeff_c;

    if (discriminant < static_cast<T>(0)) {
        return {std::nullopt, std::nullopt};
    }

    // Scale-aware tolerance: the discriminant's natural scale is set by
    // coeff_b^2 (and coeff_a*coeff_c), so an absolute machine-epsilon
    // comparison is only meaningful relative to that scale.
    const T tangent_eps =
        std::numeric_limits<T>::epsilon() *
        std::max<T>({T(1), coeff_b * coeff_b, std::abs(coeff_a * coeff_c)});

    const T sqrt_d = std::sqrt(discriminant);

    if (std::abs(discriminant) < tangent_eps) {
        const T step = -coeff_b / (static_cast<T>(2) * coeff_a);
        return {step, std::nullopt};
    }

    const T t1 = (-coeff_b + sqrt_d) / (static_cast<T>(2) * coeff_a);
    const T t2 = (-coeff_b - sqrt_d) / (static_cast<T>(2) * coeff_a);
    return {t1, t2};
}

template <typename T>
std::optional<Ellipse<T>> Ellipsoid<T>::intersectPlane(
    const Plane<T>& plane) const {
    // Ellipsoid semi-axis lengths
    const T dim_a = m_dimensions[0];
    const T dim_b = m_dimensions[1];
    const T dim_c = m_dimensions[2];

    // Validate dimensions
    if (dim_a <= T(0) || dim_b <= T(0) || dim_c <= T(0)) {
        return std::nullopt;
    }

    const T a_sq = dim_a * dim_a;
    const T b_sq = dim_b * dim_b;
    const T c_sq = dim_c * dim_c;

    // Plane coefficients: Ax + By + Cz + D = 0
    const auto coeffs = plane.getCoefficients();
    const T coeff_a = coeffs[0];
    const T coeff_b = coeffs[1];
    const T coeff_c = coeffs[2];
    const T coeff_d = coeffs[3];

    // Validate plane normal
    const T n_sq = coeff_a * coeff_a + coeff_b * coeff_b + coeff_c * coeff_c;
    const T n_len = std::sqrt(n_sq);

    const T machine_eps = std::numeric_limits<T>::epsilon();

    if (n_len <= machine_eps) {
        return std::nullopt;
    }

    // Unit normal
    Vector<T, 3> normal = plane.getNormalVector();
    normal.normalizeInPlace();

    // Signed distance from ellipsoid center to plane
    const T signed_dist = (coeff_a * m_origin[0] + coeff_b * m_origin[1] +
                           coeff_c * m_origin[2] + coeff_d) /
                          n_len;

    // Foot of perpendicular from ellipsoid center to plane
    const T px = -normal[0] * signed_dist;
    const T py = -normal[1] * signed_dist;
    const T pz = -normal[2] * signed_dist;

    // Build orthonormal basis (u, v) spanning the plane
    Vector<T, 3> vec_u;
    if (std::abs(normal[0]) < T(0.9)) {
        vec_u = crossProduct(Vector<T, 3>{T(1), T(0), T(0)}, normal);
    } else {
        vec_u = crossProduct(Vector<T, 3>{T(0), T(1), T(0)}, normal);
    }
    vec_u.normalizeInPlace();

    Vector<T, 3> vec_v = crossProduct(normal, vec_u);
    vec_v.normalizeInPlace();

    // Restricted quadratic form Q
    const T q_uu = vec_u[0] * vec_u[0] / a_sq + vec_u[1] * vec_u[1] / b_sq +
                   vec_u[2] * vec_u[2] / c_sq;

    const T q_vv = vec_v[0] * vec_v[0] / a_sq + vec_v[1] * vec_v[1] / b_sq +
                   vec_v[2] * vec_v[2] / c_sq;

    const T q_uv = vec_u[0] * vec_v[0] / a_sq + vec_u[1] * vec_v[1] / b_sq +
                   vec_u[2] * vec_v[2] / c_sq;

    // Linear terms
    const T r_u =
        px * vec_u[0] / a_sq + py * vec_u[1] / b_sq + pz * vec_u[2] / c_sq;

    const T r_v =
        px * vec_v[0] / a_sq + py * vec_v[1] / b_sq + pz * vec_v[2] / c_sq;

    // Constant term
    const T k_factor = px * px / a_sq + py * py / b_sq + pz * pz / c_sq;

    // Solve Q * [s0, t0]^T = -[r_u, r_v]^T
    const T det_q = q_uu * q_vv - q_uv * q_uv;

    // Scale-aware tolerance
    const T det_eps =
        machine_eps *
        std::max<T>({T(1), std::abs(q_uu), std::abs(q_vv), std::abs(q_uv)});

    if (det_q <= det_eps) {
        return std::nullopt;
    }

    const T s0 = (-r_u * q_vv + r_v * q_uv) / det_q;
    const T t0 = (-r_v * q_uu + r_u * q_uv) / det_q;

    // Reduced constant after completing the square
    const T k_reduced = k_factor - T(1) - (r_u * s0 + r_v * t0);

    const T eps = machine_eps *
                  std::max<T>({T(1), std::abs(k_factor), std::abs(k_reduced)});

    // No intersection
    if (k_reduced > eps) {
        return std::nullopt;
    }

    // Tangent-safe RHS
    const T rhs = std::max(T(0), -k_reduced);

    // Eigendecomposition of symmetric 2x2 matrix Q
    const T trace = q_uu + q_vv;
    const T disc = std::hypot(q_uu - q_vv, T(2) * q_uv);

    const T lambda1 = (trace + disc) / T(2);
    const T lambda2 = (trace - disc) / T(2);

    // Clamp tiny negative values caused by rounding
    const T lambda1_safe = std::max(lambda1, eps);
    const T lambda2_safe = std::max(lambda2, eps);

    // Semi-axis lengths
    T semi1 = std::sqrt(rhs / lambda1_safe);
    T semi2 = std::sqrt(rhs / lambda2_safe);

    // Axis directions
    Vector<T, 3> axis1;
    Vector<T, 3> axis2;

    if (std::abs(q_uv) <= eps) {
        // Already diagonal in (u, v)
        if (q_uu <= q_vv) {
            axis1 = vec_u;
            axis2 = vec_v;
        } else {
            axis1 = vec_v;
            axis2 = vec_u;
            std::swap(semi1, semi2);
        }
    } else {
        const T angle = T(0.5) * std::atan2(T(2) * q_uv, q_uu - q_vv);

        axis1 = std::cos(angle) * vec_u + std::sin(angle) * vec_v;

        axis2 = -std::sin(angle) * vec_u + std::cos(angle) * vec_v;

        axis1.normalizeInPlace();
        axis2.normalizeInPlace();
    }

    // Ensure semi1 is the major axis
    if (semi1 < semi2) {
        std::swap(semi1, semi2);
        std::swap(axis1, axis2);
    }

    // Ellipse center
    const Point<T, 3> center{
        m_origin[0] + px + s0 * vec_u[0] + t0 * vec_v[0],
        m_origin[1] + py + s0 * vec_u[1] + t0 * vec_v[1],
        m_origin[2] + pz + s0 * vec_u[2] + t0 * vec_v[2],
    };

    return Ellipse<T>{center, {axis1, axis2}, {semi1, semi2}, normal};
}

template <typename T>
std::optional<T> Ellipsoid<T>::arcLength(const Point<T, 3>& point_1,
                                         const Point<T, 3>& point_2) const {
    const auto eps = std::numeric_limits<T>::epsilon();
    const auto colinear = areCollinear(eps, point_1, point_2, m_origin);

    Plane<T> plane;
    if (colinear) {
        // use the xy plane instead
        plane = Plane<T>(0, 0, 1, 0);
    } else {
        plane = Plane<T>(point_1, point_2, m_origin);
    }

    const auto ellipse = intersectPlane(plane);
    if (!ellipse.has_value()) {
        return std::nullopt;
    }

    return ellipse->arcLength(point_1, point_2);
}

template <typename T>
std::array<std::optional<Path<T>>, 2> Ellipsoid<T>::tracePath(
    const Point<T, 3>& point_outside, const Point<T, 3>& point_on) const {
    const auto eps = std::numeric_limits<T>::epsilon();

    const T dim_a = m_dimensions[0];
    const T dim_b = m_dimensions[1];
    const T dim_c = m_dimensions[2];

    const T dim_a_sq = dim_a * dim_a;
    const T dim_b_sq = dim_b * dim_b;
    const T dim_c_sq = dim_c * dim_c;

    // Visibility test: is point_on directly visible (unobstructed) from
    // point_outside? True iff point_outside lies on/outside the tangent
    // plane of the ellipsoid at point_on. Equivalent to evaluating the
    // polar plane of point_on at point_outside.
    const T visibility = (point_on[0] * point_outside[0]) / dim_a_sq +
                         (point_on[1] * point_outside[1]) / dim_b_sq +
                         (point_on[2] * point_outside[2]) / dim_c_sq;

    if (visibility >= static_cast<T>(1) - eps) {
        Path<T> p1;
        p1.source2tanpoint = Vector<T, 3>(point_outside, point_on);
        p1.arclength = 0;
        p1.straight_length = p1.source2tanpoint.magnitude();
        p1.total_length = p1.straight_length;
        p1.tangent_point = point_on;

        // Unobstructed: only one physical path exists.
        std::array<std::optional<Path<T>>, 2> paths{p1, std::nullopt};
        return paths;
    }

    // Degenerate case: point_outside, m_origin, point_on exactly collinear
    // with m_origin between them (obstructed, but point_plane below would
    // be undefined since three collinear points don't define a plane).
    // Infinitely many tangent-plane choices are equally valid here; no
    // single "correct" answer exists without an extra convention.
    const auto colinear =
        areCollinear<T, 3>(eps, m_origin, point_on, point_outside);

    Plane<T> point_plane;

    if (colinear) {
        // Degenerate: point_outside, m_origin, point_on exactly collinear with
        // m_origin between them. Infinitely many planes contain this line; pick
        // one deterministically (least-aligned standard axis as helper vector)
        // rather than returning no path at all — callers depend on paths[0]
        // always being populated.
        Vector<T, 3> dir(point_on, point_outside);
        dir.normalizeInPlace();

        Vector<T, 3> helper{0, 0, 0};
        const T ax = std::abs(dir[0]);
        const T ay = std::abs(dir[1]);
        const T az = std::abs(dir[2]);

        if (ax <= ay && ax <= az) {
            helper = Vector<T, 3>{1, 0, 0};
        } else if (ay <= ax && ay <= az) {
            helper = Vector<T, 3>{0, 1, 0};
        } else {
            helper = Vector<T, 3>{0, 0, 1};
        }

        Vector<T, 3> normal = crossProduct(dir, helper);
        normal.normalizeInPlace();

        point_plane =
            Plane<T>(normal[0],
                     normal[1],
                     normal[2],
                     -(normal[0] * m_origin[0] + normal[1] * m_origin[1] +
                       normal[2] * m_origin[2]));
    } else {
        point_plane = Plane<T>(m_origin, point_on, point_outside);
    }
    point_plane.normalize();

    const T point_out_x = point_outside[0];
    const T point_out_y = point_outside[1];
    const T point_out_z = point_outside[2];

    const T polar_a = point_out_x / dim_a_sq;
    const T polar_b = point_out_y / dim_b_sq;
    const T polar_c = point_out_z / dim_c_sq;

    Plane<T> polar_plane(polar_a, polar_b, polar_c, static_cast<T>(-1));
    polar_plane.normalize();

    const auto intersection_line = intersectPlanes(polar_plane, point_plane);

    if (!intersection_line) {
        RTB_DEBUG_LOG("No valid intersection between the planes!\n");
#ifndef NDEBUG
        polar_plane.print();
        point_plane.print();
#endif
        return {std::nullopt, std::nullopt};
    }

    const auto intersections = intersectRay(intersection_line.value());
    const std::optional<T>& intersection_a = intersections[0];
    const std::optional<T>& intersection_b = intersections[1];

    if (!intersection_a.has_value() || !intersection_b.has_value()) {
        RTB_DEBUG_LOG("No valid intersection between ray and ellipsoid!\n");
        return {std::nullopt, std::nullopt};
    }

    const Point<T, 3> tangent_a =
        intersection_line->getPosition(intersection_a.value());

    const Point<T, 3> tangent_b =
        intersection_line->getPosition(intersection_b.value());

    const auto ellipse = intersectPlane(point_plane);

    if (!ellipse) {
        RTB_DEBUG_LOG("No valid ellipse!\n");
        return {std::nullopt, std::nullopt};
    }

    const T arc_a = ellipse->arcLength(point_on, tangent_a);
    const T arc_b = ellipse->arcLength(point_on, tangent_b);

    Path<T> p1;
    p1.source2tanpoint = Vector<T, 3>(point_outside, tangent_a);
    p1.arclength = arc_a;
    p1.straight_length = p1.source2tanpoint.magnitude();
    p1.total_length = arc_a + p1.straight_length;
    p1.tangent_point = tangent_a;

    Path<T> p2;
    p2.source2tanpoint = Vector<T, 3>(point_outside, tangent_b);
    p2.arclength = arc_b;
    p2.straight_length = p2.source2tanpoint.magnitude();
    p2.total_length = arc_b + p2.straight_length;
    p2.tangent_point = tangent_b;

    std::array<std::optional<Path<T>>, 2> paths{p1, p2};

    return paths;
}

}  // namespace RTB
#endif /* RTB_ELLIPSOID_HPP */
