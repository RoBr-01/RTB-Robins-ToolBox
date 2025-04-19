#ifndef ELLIPSOID_HPP
#define ELLIPSOID_HPP

// STL
#include <array>

// RTB
#include "Math.hpp"
#include "Plane.hpp"
#include "Ray.hpp"

namespace RTB {

// Arc length calculation mode
enum class ArcMode {
    FastEstimate,  // Use cbrt(a*b*c) * angle approximation
    Accurate       // Use slerp + numerical integration on ellipsoid surface
};

template <typename T>
struct trace_results {
    std::array<T, 2> pathlengths;
    std::array<T, 2> arclengths;
    std::array<Point<T, 3>, 2> tangents;
};

template <typename T>
class Ellipsoid {
   public:
    Ellipsoid();
    Ellipsoid(T length, T width, T height);
    Ellipsoid(std::initializer_list<T> sizes);
    ~Ellipsoid();
    std::array<T, 3> GetDimensions();

    std::array<T, 2> GetIntersection(const Ray3R &ray);
    trace_results<T> TracePathEllipsoid(const Point3R &Source);

    Vec3R scale_ellipsoid(const Vec3R &dir, double a, double b, double c);

   private:
    T ArcLengthOnEllipsoid(const Vec3R &dir_from_center_1,
                           const Vec3R &dir_from_center_2,
                           ArcMode mode = ArcMode::Accurate,
                           int steps = 100);

    T AccurateArcLength(const Vec3R &unit_start,
                        const Vec3R &unit_end,
                        double a,
                        double b,
                        double c,
                        int steps);

    Vec3R dEllipsoid_dt(const Vec3R &unit_start,
                        const Vec3R &unit_end,
                        double t,
                        double a,
                        double b,
                        double c,
                        double h = 1e-5);

   private:
    std::array<T, 3> m_dimensions;
    // Center
    // Rotation
};

template <typename T>
Ellipsoid<T>::Ellipsoid() {}

template <typename T>
Ellipsoid<T>::~Ellipsoid() {}

template <typename T>
std::array<T, 2> Ellipsoid<T>::GetIntersection(const Ray3R &ray) {
    // ray and ellipsoid intersection

    // This splitting is really inefficient
    T vx = ray.GetDirection()[x];
    T vy = ray.GetDirection()[y];
    T vz = ray.GetDirection()[z];

    T x0 = ray.GetOrigin()[x];
    T y0 = ray.GetOrigin()[y];
    T z0 = ray.GetOrigin()[z];

    T A = m_dimensions[x];
    T B = m_dimensions[y];
    T C = m_dimensions[z];

    T a = (vx / A) * (vx / A) + (vy / B) * (vy / B) + (vz / C) * (vz / C);
    T b = (2) * (((x0 * vx) / (A * A)) + ((y0 * vy) / (B * B)) +
                 ((z0 * vz) / (C * C)));
    T c = (x0 / A) * (x0 / A) + (y0 / B) * (y0 / B) + (z0 / C) * (z0 / C) - 1;

    // Calculate discriminant
    T d = b * b - 4 * a * c;

    // Early return - no valid intersection
    if (d < 0) {
        return {-1, -1};
    }

    // Only one intersection, compact calculation
    if (d == 0) {
        T t_tangent = -b / (2 * a);

        // Can't be negative or zero
        if (t_tangent <= 0) {
            return {-1, -1};
        }

        return {t_tangent, -1};
    }

    T t_one = (-b + std::sqrt(d)) / (2 * a);
    T t_two = (-b - std::sqrt(d)) / (2 * a);

    // Can't be negative or zero
    if (t_one <= 0) {
        t_one = -1;
    }

    if (t_two <= 0) {
        t_two = -1;
    }

    return {t_one, t_two};
}

// Apply ellipsoid scaling to a direction vector
template <typename T>
Vec3R Ellipsoid<T>::scale_ellipsoid(const Vec3R &dir,
                                    double a,
                                    double b,
                                    double c) {
    return Vec3R{a * dir[0], b * dir[1], c * dir[2]};
}

// Derivative of ellipsoid path at point t using central difference
template <typename T>
Vec3R Ellipsoid<T>::dEllipsoid_dt(const Vec3R &unit_start,
                                  const Vec3R &unit_end,
                                  double t,
                                  double a,
                                  double b,
                                  double c,
                                  double h = 1e-5) {
    Vec3R pt1 =
        scale_ellipsoid(Math::slerp(unit_start, unit_end, t - h), a, b, c);
    Vec3R pt2 =
        scale_ellipsoid(Math::slerp(unit_start, unit_end, t + h), a, b, c);
    return Vec3R{(pt2[0] - pt1[0]) / (2 * h),
                 (pt2[1] - pt1[1]) / (2 * h),
                 (pt2[2] - pt1[2]) / (2 * h)};
}
// Arc length via Simpson's rule
template <typename T>
T Ellipsoid<T>::AccurateArcLength(const Vec3R &unit_start,
                                  const Vec3R &unit_end,
                                  double a,
                                  double b,
                                  double c,
                                  int steps) {
    double total = 0.0;
    double h = 1.0 / steps;

    for (int i = 0; i <= steps; ++i) {
        double t = i * h;
        double weight = (i == 0 || i == steps) ? 1 : (i % 2 == 0 ? 2 : 4);
        Vec3R deriv = dEllipsoid_dt(unit_start, unit_end, t, a, b, c);
        total += weight * deriv.Length();
    }

    return (h / 3.0) * total;
}

template <typename T>
T Ellipsoid<T>::ArcLengthOnEllipsoid(const Vec3R &dir_from_center_1,
                                     const Vec3R &dir_from_center_2,
                                     ArcMode mode = ArcMode::Accurate,
                                     int steps = 100) {
    Vec3R u = unit_vector(dir_from_center_1);
    Vec3R v = unit_vector(dir_from_center_2);

    if (mode == ArcMode::FastEstimate) {
        double angle = acos(dotprod(u, v));  // radians
        double r_avg =
            std::cbrt(m_dimensions[0] * m_dimensions[1] * m_dimensions[2]);
        return r_avg * angle;
    } else {
        return AccurateArcLength(
            u, v, m_dimensions[0], m_dimensions[1], m_dimensions[2], steps);
    }
}

template <typename T>
trace_results<T> Ellipsoid<T>::TracePathEllipsoid(const Point3R &Source) {
    std::array<Point3R, 2> tangent_points;
    std::array<T, 2> pathlengths;
    std::array<T, 2> arclengths;

    // Construct the polar plane from the source point and ellipsoid
    T a1 = Source[0] / (m_dimensions[0] * m_dimensions[0]);
    T b1 = Source[1] / (m_dimensions[1] * m_dimensions[1]);
    T c1 = Source[2] / (m_dimensions[2] * m_dimensions[2]);
    PlaneR polar_plane(a1, b1, c1, -1);

    Point3R origin = {0, 0, 0};

    // Split for ease of use, can be better
    T a = m_dimensions[0];
    T b = m_dimensions[1];
    T c = m_dimensions[2];

    for (size_t Currentear = 0; Currentear < 2; Currentear++) {
        Point3R ear = m_earpositions_cart[Currentear];

        // Construct the plane defined by source, origin, and ear
        PlaneR STO_plane(origin, ear, Source);

        Ray3R intersection_line = IntersectPlanes(polar_plane, STO_plane);

        Vec3R N = intersection_line.GetDirection();
        Point3R P = intersection_line.GetOrigin();

        // Splits for formulas
        T Nx = N[0], Ny = N[1], Nz = N[2];
        T Px = P[0], Py = P[1], Pz = P[2];

        // Ray-ellipsoid intersection
        T A = (Nx * Nx) / (a * a) + (Ny * Ny) / (b * b) + (Nz * Nz) / (c * c);

        T B = 2 *
              ((Px * Nx) / (a * a) + (Py * Ny) / (b * b) + (Pz * Nz) / (c * c));

        T C =
            (Px * Px) / (a * a) + (Py * Py) / (b * b) + (Pz * Pz) / (c * c) - 1;

        T D = B * B - 4 * A * C;

        if (D < 0) {
            std::cerr << "No intersection was found, this should be "
                         "impossible!\n";
            std::exit(1);
        }

        T t_plus = (-B + sqrt(D)) / (2 * A);
        T t_minus = (-B - sqrt(D)) / (2 * A);

        Point3R tangent_one = intersection_line.GetPosition(t_plus);
        Point3R tangent_two = intersection_line.GetPosition(t_minus);

        // Calculate the arc angles between the ear and both tangents
        Vec3R vecEar(origin, ear);
        // vecEar = unit_vector(vecEar);

        Vec3R tangent_one_vec(origin, tangent_one);
        // tangent_one_vec = unit_vector(tangent_one_vec);

        Vec3R tangent_two_vec(origin, tangent_two);
        // tangent_two_vec = unit_vector(tangent_two_vec);

        // For future reference:
        // Up until here everything is happy happy joyjoy, but now comes the
        // kicker: finding the arclength along the ellipsoid (more
        // specifically the geodesic), is non-trivial... Meaning we can't
        // exactly calculate it, only approximate it. Usual approximations
        // like vincenty's method assume rotational symmetry along at least
        // one axis But since our head is triaxial (3 distinct axes) we need
        // another method, that's the "ArcLengthOnEllipsoid" function below

        // Calculate the arclength directly
        double length_one = ArcLengthOnEllipsoid(
            vecEar, tangent_one_vec, ArcMode::Accurate, 100);

        double length_two = ArcLengthOnEllipsoid(
            vecEar, tangent_two_vec, ArcMode::Accurate, 100);

        // T arc_angle_one = acosd(
        //     dotprod(unit_vector(vecEar), unit_vector(tangent_one_vec)));
        // T arc_angle_two = acosd(
        //     dotprod(unit_vector(vecEar), unit_vector(tangent_two_vec)));
        double arclength;
        Point3R closest_tangent;
        double pathlength;
        double tangent_one_path = distance_2points(Source, tangent_one);
        double tangent_two_path = distance_2points(Source, tangent_two);

        // Determine the closer tangent point
        if (length_one <= length_two) {
            arclength = length_one;
            closest_tangent = tangent_one;
            pathlength = tangent_one_path;
        } else {
            arclength = length_two;
            closest_tangent = tangent_two;
            pathlength = tangent_two_path;
        }

        double straightpath = distance_2points(Source, origin);

        if (pathlength < straightpath) {
            pathlengths[Currentear] = pathlength + arclength;
            arclengths[Currentear] = arclength;
            tangent_points[Currentear] = closest_tangent;

        } else {
            pathlengths[Currentear] = straightpath;
            arclengths[Currentear] = 0;
            tangent_points[Currentear] = ear;
        }
    }
    return {pathlengths, arclengths, tangent_points};
}

}  // namespace RTB

#endif /* ELLIPSOID_HPP */
