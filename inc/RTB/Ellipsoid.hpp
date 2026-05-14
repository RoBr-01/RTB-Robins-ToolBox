#ifndef ELLIPSOID_HPP
#define ELLIPSOID_HPP

#include <RTB/Ellipse.hpp>
#include <RTB/Math.hpp>
#include <RTB/Plane.hpp>
#include <RTB/Point.hpp>
#include <array>

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

template <typename T>
struct Path {
    T total_length{};
    T straigh_length{};
    T arclength{};
    Point<T, 3> tangent_point{};
    // source to tangent point (unnormalised)
    Vector<T, 3> source2tanpoint{};
};

template <typename T>
class Ellipsoid {
   public:
    Ellipsoid() = default;
    Ellipsoid(T length, T width, T height);

    [[nodiscard]] std::array<T, 3> getDimensions() const;

    [[nodiscard]] std::array<std::optional<T>, 2> intersectRay(
        Ray<T, 3> ray) const;

    [[nodiscard]] std::optional<Ellipse<T>> intersectPlane(
        Plane<T> plane) const;

    // Basically the arclength of the ellipse defined by intersecting the plane
    // formed by point_1, point_2 and the origin with the ellipsoid
    [[nodiscard]] T arcLength(const Point<T, 3>& point_1,
                              const Point<T, 3>& point_2) const;

    [[nodiscard]] std::array<std::optional<Path<T>>, 2> tracePath(
        Point<T, 3> point_outside, Point<T, 3> point_on) const;

   private:
    std::array<T, 3> m_dimensions{};
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
std::array<std::optional<T>, 2> Ellipsoid<T>::intersectRay(
    const Ray<T, 3> ray) const {
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

    const T sqrt_d = std::sqrt(discriminant);

    if (std::abs(discriminant) < std::numeric_limits<T>::epsilon()) {
        const T step = -coeff_b / (static_cast<T>(2) * coeff_a);
        return {step, std::nullopt};
    }

    const T t1 = (-coeff_b + sqrt_d) / (static_cast<T>(2) * coeff_a);
    const T t2 = (-coeff_b - sqrt_d) / (static_cast<T>(2) * coeff_a);
    return {t1, t2};
}

template <typename T>
std::optional<Ellipse<T>> Ellipsoid<T>::intersectPlane(
    const Plane<T> plane) const {
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
    const Point<T, 3> center{m_origin[0] + px + s0 * vec_u[0] + t0 * vec_v[0],
                             m_origin[1] + py + s0 * vec_u[1] + t0 * vec_v[1],
                             m_origin[2] + pz + s0 * vec_u[2] + t0 * vec_v[2]};

    return Ellipse<T>{center, {axis1, axis2}, {semi1, semi2}, normal};
}

template <typename T>
T Ellipsoid<T>::arcLength(const Point<T, 3>& point_1,
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

    T arclength{};
    if (ellipse.has_value()) {
        arclength = ellipse->arcLength(point_1, point_2);
    } else {
        // TODO: graceful exit somehow?
        std::abort();
    }

    return arclength;
}

template <typename T>
std::array<std::optional<Path<T>>, 2> Ellipsoid<T>::tracePath(
    const Point<T, 3> point_outside, const Point<T, 3> point_on) const {
    const auto eps = std::numeric_limits<T>::epsilon();

    const auto colinear =
        areCollinear<T, 3>(eps, m_origin, point_on, point_outside);

    // FIXME: colinear can mean 2 things:
    // -either the point_on is in between point_outside and m_origin
    // - or m_origin is between point_on and point_outside
    // Both have different solutions!
    if (colinear) {
        Path<T> p1;
        p1.source2tanpoint = Vector<T, 3>(point_outside, point_on);
        p1.arclength = 0;
        p1.straigh_length = p1.source2tanpoint.magnitude();
        p1.total_length = p1.straigh_length;
        p1.tangent_point = point_on;

        std::array<std::optional<Path<T>>, 2> paths{p1, p1};
        return paths;
    }

    Plane<T> point_plane(m_origin, point_on, point_outside);
    point_plane.normalize();

    const T dim_a = m_dimensions[0];
    const T dim_b = m_dimensions[1];
    const T dim_c = m_dimensions[2];

    const T dim_a_sq = dim_a * dim_a;
    const T dim_b_sq = dim_b * dim_b;
    const T dim_c_sq = dim_c * dim_c;

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
        std::cerr << "No valid intersection between the planes!\n";
        polar_plane.print();
        point_plane.print();
        std::abort();
    }

    const auto intersections = intersectRay(intersection_line.value());

    if (!intersections[0] || !intersections[1]) {
        std::cerr << "No valid intersection between ray and ellipsoid!\n";
        std::abort();
    }

    const Point<T, 3> tangent_a =
        intersection_line->getPosition(intersections[0].value());

    const Point<T, 3> tangent_b =
        intersection_line->getPosition(intersections[1].value());

    const auto ellipse = intersectPlane(point_plane);

    if (!ellipse) {
        std::cerr << "No valid ellipse!\n";
        std::abort();
    }

    const T arc_a = ellipse->arcLength(point_on, tangent_a);
    const T arc_b = ellipse->arcLength(point_on, tangent_b);

    Path<T> p1;
    p1.source2tanpoint = Vector<T, 3>(point_outside, tangent_a);
    p1.arclength = arc_a;
    p1.straigh_length = p1.source2tanpoint.magnitude();
    p1.total_length = arc_a + p1.straigh_length;
    p1.tangent_point = tangent_a;

    Path<T> p2;
    p2.source2tanpoint = Vector<T, 3>(point_outside, tangent_b);
    p2.arclength = arc_b;
    p2.straigh_length = p2.source2tanpoint.magnitude();
    p2.total_length = arc_b + p2.straigh_length;
    p2.tangent_point = tangent_b;

    std::array<std::optional<Path<T>>, 2> paths{p1, p2};

    return paths;
}

}  // namespace RTB
#endif /* ELLIPSOID_HPP */
