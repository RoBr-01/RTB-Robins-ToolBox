#ifndef ELLIPSOID_HPP
#define ELLIPSOID_HPP

// STL
#include <array>
#include <cassert>
#include <cmath>
#include <limits>
#include <optional>

// RTB
#include <RTB/Math.hpp>
#include <RTB/Plane.hpp>
#include <RTB/Point.hpp>
#include <RTB/Ray.hpp>
#include <RTB/Vector.hpp>

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
 * @brief dim_a single diffraction path from source to one ear.
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
    T theta;  // parametric angle [0, 2*pi)
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
    Ellipsoid(std::initializer_list<T> sizes);
    explicit Ellipsoid(const std::array<T, 3>& sizes);

    [[nodiscard]] const std::array<T, 3>& getDimensions() const;

    /**
     * @brief Ray-ellipsoid intersection.
     *
     * Returns the two ray parameters t1, t2 at which the ray intersects,
     * or std::nullopt if there is no intersection.
     * For a tangent intersection t1 == t2.
     */
    [[nodiscard]] std::optional<std::array<T, 2>> intersectRay(
        const Ray<T, 3>& ray) const;

    /**
     * @brief Plane-ellipsoid intersection.
     *
     * Returns the ellipse formed by the intersection.
     * Assumes the plane is normalized.
     */
    [[nodiscard]] EllipseParams<T> intersectPlane(const Plane<T>& plane) const;

    /**
     * @brief Traces the diffraction path from a source point to two ear points
     *        around the ellipsoid surface.
     */
    [[nodiscard]] TraceResults<T> tracePath(
        const Point<T, 3>& source,
        const std::array<Point<T, 3>, 2>& ears) const;

   private:
    /**
     * @brief Returns the arc length along the ellipse cross-section between
     *        p1 and p2, using the compile-time selected Method.
     */
    [[nodiscard]] T arcLength(const Point<T, 3>& p1,
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
const std::array<T, 3>& Ellipsoid<T, Method>::getDimensions() const {
    return m_dimensions;
}

template <typename T, ArcLengthMethod Method>
std::optional<std::array<T, 2>> Ellipsoid<T, Method>::intersectRay(
    const Ray<T, 3>& ray) const {
    const Vector<T, 3> dir = ray.getDirection();
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
        return std::nullopt;
    }

    const T sqrt_d = std::sqrt(discriminant);
    const T t1 = (-coeff_b + sqrt_d) / (static_cast<T>(2) * coeff_a);
    const T t2 = (-coeff_b - sqrt_d) / (static_cast<T>(2) * coeff_a);
    return std::array<T, 2>{t1, t2};
}

template <typename T, ArcLengthMethod Method>
EllipseParams<T> Ellipsoid<T, Method>::intersectPlane(
    const Plane<T>& plane) const {
    EllipseParams<T> result;

    const T dim_a = m_dimensions[0];
    const T dim_b = m_dimensions[1];
    const T dim_c = m_dimensions[2];
    const T a_sq = dim_a * dim_a;
    const T b_sq = dim_b * dim_b;
    const T c_sq = dim_c * dim_c;

    const auto coeffs = plane.getCoefficients();
    const T coeff_a = coeffs[0];
    const T coeff_b = coeffs[1];
    const T coeff_c = coeffs[2];
    const T coeff_d = coeffs[3];
    const T n_sq = coeff_a * coeff_a + coeff_b * coeff_b + coeff_c * coeff_c;

    result.center = Point<T, 3>{-coeff_a * coeff_d / n_sq,
                                -coeff_b * coeff_d / n_sq,
                                -coeff_c * coeff_d / n_sq};
    result.normal = plane.getNormalVector();
    result.normal.normalizeInPlace();

    // Build orthonormal basis in the plane
    Vector<T, 3> vec_u;
    Vector<T, 3> vec_v;
    if (std::abs(coeff_a) < static_cast<T>(0.9)) {
        vec_u = crossProduct(Vector<T, 3>{1, 0, 0}, result.normal);
    } else {
        vec_u = crossProduct(Vector<T, 3>{0, 1, 0}, result.normal);
    }
    vec_u.normalizeInPlace();
    vec_v = crossProduct(result.normal, vec_u);

    // Project ellipsoid metric onto the plane basis to get 2D quadratic form
    const T q_uu = vec_u[0] * vec_u[0] / a_sq + vec_u[1] * vec_u[1] / b_sq +
                   vec_u[2] * vec_u[2] / c_sq;
    const T q_vv = vec_v[0] * vec_v[0] / a_sq + vec_v[1] * vec_v[1] / b_sq +
                   vec_v[2] * vec_v[2] / c_sq;
    const T q_uv = vec_u[0] * vec_v[0] / a_sq + vec_u[1] * vec_v[1] / b_sq +
                   vec_u[2] * vec_v[2] / c_sq;

    // Eigendecomposition of the 2x2 quadratic form
    const T trace = q_uu + q_vv;
    const T det = q_uu * q_vv - q_uv * q_uv;
    const T discriminant = std::sqrt(trace * trace - static_cast<T>(4) * det);
    const T lambda1 = (trace + discriminant) / static_cast<T>(2);
    const T lambda2 = (trace - discriminant) / static_cast<T>(2);

    result.semi_axis_lengths[0] = static_cast<T>(1) / std::sqrt(lambda1);
    result.semi_axis_lengths[1] = static_cast<T>(1) / std::sqrt(lambda2);

    const T angle =
        static_cast<T>(0.5) * std::atan2(static_cast<T>(2) * q_uv, q_uu - q_vv);
    result.semi_axes[0] = std::cos(angle) * vec_u + std::sin(angle) * vec_v;
    result.semi_axes[1] = -std::sin(angle) * vec_u + std::cos(angle) * vec_v;

    return result;
}

// ==============================
// Arc length helper (non-member)
// ==============================

template <typename T>
LocalCoords<T> transformToLocalCoords(const Point<T, 3>& point,
                                      const EllipseParams<T>& ellipse) {
    LocalCoords<T> result;

    const Vector<T, 3> translated{point[0] - ellipse.center[0],
                                  point[1] - ellipse.center[1],
                                  point[2] - ellipse.center[2]};

    result.u = dotProduct(translated, ellipse.semi_axes[0]) /
               ellipse.semi_axis_lengths[0];
    result.v = dotProduct(translated, ellipse.semi_axes[1]) /
               ellipse.semi_axis_lengths[1];

    result.theta = std::atan2(result.v, result.u);
    if (result.theta < static_cast<T>(0)) {
        result.theta += static_cast<T>(2) * static_cast<T>(pi);
    }

    return result;
}

template <typename T, ArcLengthMethod Method>
T Ellipsoid<T, Method>::arcLength(const Point<T, 3>& p1,
                                  const Point<T, 3>& p2,
                                  const Plane<T>& plane) const {
    const EllipseParams<T> ellipse = intersectPlane(plane);

    const LocalCoords<T> local1 = transformToLocalCoords(p1, ellipse);
    const LocalCoords<T> local2 = transformToLocalCoords(p2, ellipse);

    const T axis_a = ellipse.semi_axis_lengths[0];
    const T axis_b = ellipse.semi_axis_lengths[1];

    T t1 = local1.theta;
    T t2 = local2.theta;

    // Shortest arc
    T angle_diff = t2 - t1;
    if (angle_diff < static_cast<T>(0)) {
        angle_diff += static_cast<T>(2) * static_cast<T>(pi);
    }
    if (angle_diff > static_cast<T>(pi)) {
        angle_diff = static_cast<T>(2) * static_cast<T>(pi) - angle_diff;
        std::swap(t1, t2);
    }

    if constexpr (Method == ArcLengthMethod::Polynomial) {
        // 4th-degree polynomial approximation.
        // Reasonable for near-circular ellipses (b/a > ~0.8),
        // degrades for high eccentricity.
        const T ratio = axis_b / axis_a;
        const T ratio_sq = ratio * ratio;
        const T ratio_cu = ratio_sq * ratio;
        const T ratio_qu = ratio_sq * ratio_sq;
        return angle_diff * axis_a *
               (static_cast<T>(1) + static_cast<T>(0.25) * ratio_sq -
                static_cast<T>(0.125) * ratio_cu +
                static_cast<T>(0.0625) * ratio_qu);
    } else if constexpr (Method == ArcLengthMethod::Ramanujan) {
        // Ramanujan's second approximation for full ellipse perimeter,
        // scaled by angle_diff / (2*pi) for axis_a partial arc.
        // Error ~0.0000003% for typical ellipses.
        const T ratio = ((axis_a - axis_b) * (axis_a - axis_b)) /
                        ((axis_a + axis_b) * (axis_a + axis_b));
        const T perimeter =
            static_cast<T>(pi) * (axis_a + axis_b) *
            (static_cast<T>(1) +
             (static_cast<T>(3) * ratio) /
                 (static_cast<T>(10) +
                  std::sqrt(static_cast<T>(4) - static_cast<T>(3) * ratio)));
        return perimeter *
               (angle_diff / (static_cast<T>(2) * static_cast<T>(pi)));
    } else {
        // 10-point Gauss-Legendre quadrature.
        // Integrates sqrt(a²sin²t + b²cos²t) over [t1, t1+angle_diff].
        // Effectively full floating-point precision for any eccentricity.

        // Abscissae and weights for 10-point Gauss-Legendre on [-1, 1]

        static std::array<double, 10> gl_x{-0.9739065285,
                                           -0.8650633667,
                                           -0.6794095683,
                                           -0.4333953941,
                                           -0.1488743390,
                                           0.1488743390,
                                           0.4333953941,
                                           0.6794095683,
                                           0.8650633667,
                                           0.9739065285};

        static std::array<double, 10> gl_w = {0.0666713443,
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
        for (std::size_t i = 0; i < 10; ++i) {
            const T angle = mid + half_range * static_cast<T>(gl_x[i]);
            const T sin_t = std::sin(angle);
            const T cos_t = std::cos(angle);
            const T integrand = std::sqrt(axis_a * axis_a * sin_t * sin_t +
                                          axis_b * axis_b * cos_t * cos_t);
            integral += static_cast<T>(gl_w[i]) * integrand;
        }

        return half_range * integral;
    }
}

// ==============================
// TracePath
// ==============================

template <typename T, ArcLengthMethod Method>
TraceResults<T> Ellipsoid<T, Method>::tracePath(
    const Point<T, 3>& source, const std::array<Point<T, 3>, 2>& ears) const {
    TraceResults<T> results{};

    const T dim_a = m_dimensions[0];
    const T dim_b = m_dimensions[1];
    const T dim_c = m_dimensions[2];

    const T a_sq = dim_a * dim_a;
    const T b_sq = dim_b * dim_b;
    const T c_sq = dim_c * dim_c;
    const T source_x = source[0];
    const T source_y = source[1];
    const T source_z = source[2];

    const T epsilon = std::sqrt(std::numeric_limits<T>::epsilon());

    // Source must be strictly outside the ellipsoid
    assert(source_x * source_x / a_sq + source_y * source_y / b_sq +
                   source_z * source_z / c_sq >
               static_cast<T>(1) + epsilon &&
           "Source must be strictly outside the ellipsoid.");

    // Polar plane of the source point w.r.t. the ellipsoid:
    // (source_x/A²)x + (source_y/B²)y + (source_z/C²)z = 1
    T polar_a = source_x / a_sq;
    T polar_b = source_y / b_sq;
    T polar_c = source_z / c_sq;

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
    polar_plane.normalize();

    // Projects a point radially onto the ellipsoid surface.
    // Corrects for floating-point drift — ears are expected to lie on the
    // surface, so we enforce it exactly before using them geometrically.
    // Note: radial projection from origin, not closest-point (quartic).
    // For points already near the surface the difference is negligible.
    auto project_onto_surface = [&](const Point<T, 3>& point) -> Point<T, 3> {
        const T scale =
            std::sqrt(point[0] * point[0] / a_sq + point[1] * point[1] / b_sq +
                      point[2] * point[2] / c_sq);
        return Point<T, 3>{
            point[0] / scale, point[1] / scale, point[2] / scale};
    };

    // Computes a single EarPath given a tangent point and ear.
    auto make_ear_path = [&](const Point<T, 3>& tangent,
                             const Point<T, 3>& ear,
                             const Plane<T>& sto_plane) -> EarPath<T> {
        EarPath<T> path;
        path.tangent_point = tangent;
        path.direct_ray = Vector<T, 3>(source, tangent);
        const T direct_length = path.direct_ray.magnitude();
        path.arclength = arcLength(tangent, ear, sto_plane);
        path.pathlength = direct_length + path.arclength;
        return path;
    };

    for (std::size_t ear_idx = 0; ear_idx < 2; ++ear_idx) {
        const Point<T, 3> ear = project_onto_surface(ears[ear_idx]);
        std::array<EarPath<T>, 2>& ear_paths =
            (ear_idx == 0) ? results.left_ear_paths : results.right_ear_paths;

        // Check if the straight source-to-ear path is occluded.
        // to_ear is unnormalised — t=1 corresponds exactly to the ear position,
        // so the occlusion window is (epsilon, 1-epsilon) in ray-parameter
        // space.
        const Vector<T, 3> to_ear(source, ear);
        const Ray<T, 3> source_to_ear(source, to_ear);

        const auto straight_intersections = intersectRay(source_to_ear);
        bool path_clear = true;
        if (straight_intersections.has_value()) {
            for (const T step : straight_intersections.value()) {
                if (step > epsilon && step < static_cast<T>(1) - epsilon) {
                    path_clear = false;
                    break;
                }
            }
        }

        if (path_clear) {
            // No occlusion — both path slots carry the direct path.
            EarPath<T> direct{};
            direct.pathlength = to_ear.magnitude();
            direct.arclength = static_cast<T>(0);
            direct.tangent_point = ear;
            direct.direct_ray = to_ear;
            ear_paths[0] = direct;
            ear_paths[1] = direct;
            continue;
        }

        // Diffraction plane through origin, ear, and source
        Plane<T> sto_plane(m_origin, ear, source);
        sto_plane.normalize();

        // Intersection of polar plane and diffraction plane gives the tangent
        // line
        const auto intersection_line = intersectPlanes(polar_plane, sto_plane);
        assert(intersection_line.has_value() &&
               "Polar plane and diffraction plane are parallel — degenerate "
               "geometry.");

        const auto intersections = intersectRay(intersection_line.value());
        assert(
            intersections.has_value() &&
            "Tangent line does not intersect ellipsoid — degenerate geometry.");

        const Point<T, 3> tangent_one =
            intersection_line->getPosition(intersections.value()[0]);
        const Point<T, 3> tangent_two =
            intersection_line->getPosition(intersections.value()[1]);

        // Compute both paths and sort: [0] shorter, [1] longer
        const EarPath<T> path_one = make_ear_path(tangent_one, ear, sto_plane);
        const EarPath<T> path_two = make_ear_path(tangent_two, ear, sto_plane);

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