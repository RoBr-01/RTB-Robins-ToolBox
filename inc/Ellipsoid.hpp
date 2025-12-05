#ifndef ELLIPSOID_HPP
#define ELLIPSOID_HPP

// STL
#include <array>
#include <source_location>

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
class Ellipsoid {
   public:
    Ellipsoid();
    Ellipsoid(T length, T width, T height);
    Ellipsoid(std::initializer_list<T> sizes);
    Ellipsoid(std::array<T, 3> sizes);
    ~Ellipsoid() = default;
    const std::array<T, 3>& GetDimensions() const;

    std::array<T, 2> Intersect(const Ray<T, 3>& ray) const;
    trace_results<T> TracePath(const Point<T, 3>& Source,
                               const std::array<Point<T, 3>, 2>& ears) const;

   private:
   private:
    std::array<T, 3> m_dimensions;
    Point<T, 3> m_origin = {0, 0, 0};
    // Rotation
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

// template <typename T>
// std::array<T, 2> Ellipsoid<T>::Intersect(const Ray<T, 3>& ray) const {
//     auto direction = ray.GetDirection();

//     // This splitting is really inefficient
//     T vx = direction[0];
//     T vy = direction[1];
//     T vz = direction[2];

//     auto origin = ray.GetOrigin();

//     T x0 = origin[0];
//     T y0 = origin[1];
//     T z0 = origin[2];

//     T A = m_dimensions[0];
//     T B = m_dimensions[1];
//     T C = m_dimensions[2];

//     T a = (vx / A) * (vx / A) + (vy / B) * (vy / B) + (vz / C) * (vz / C);
//     T b = (2) * (((x0 * vx) / (A * A)) + ((y0 * vy) / (B * B)) +
//                  ((z0 * vz) / (C * C)));
//     T c = (x0 / A) * (x0 / A) + (y0 / B) * (y0 / B) + (z0 / C) * (z0 / C) -
//     1;

//     // Calculate discriminant
//     T d = b * b - 4 * a * c;

//     // Early return - no valid intersection
//     if (d < 0) {
//         return {-1, -1};
//     }

//     // Only one intersection, compact calculation
//     if (d == 0) {
//         T t_tangent = -b / (2 * a);

//         // Can't be negative or zero
//         if (t_tangent <= 0) {
//             return {-1, -1};
//         }

//         return {t_tangent, -1};
//     }

//     T t_one = (-b + std::sqrt(d)) / (2 * a);
//     T t_two = (-b - std::sqrt(d)) / (2 * a);

//     return {t_one, t_two};
// }

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
trace_results<T> Ellipsoid<T>::TracePath(
    const Point<T, 3>& source, const std::array<Point<T, 3>, 2>& ears) const {
    trace_results<T> results;
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
        constexpr T epsilon = std::numeric_limits<T>::epsilon() * 100;
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
            results.tangent_points[Currentear] = ear;
            results.direct_rays[Currentear] = to_ear;
            results.pathlengths[Currentear] = ear_distance;
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

        // TODO: this comparison needs to be very robust!
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
        // Arc length :(
        // T arc_length = Arclength(results.tangent_points[Currentear], ear);
        T arc_length = 0;
        results.arclengths[Currentear] = arc_length;
        results.pathlengths[Currentear] = direct_length + arc_length;
    }

    return results;
}

}  // namespace RTB

#endif /* ELLIPSOID_HPP */
