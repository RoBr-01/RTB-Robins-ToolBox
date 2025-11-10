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
    ~Ellipsoid();
    const std::array<T, 3>& GetDimensions() const;

    std::array<T, 2> GetIntersection(const Ray<T, 3>& ray);
    trace_results<T> TracePathEllipsoid(
        const Point<T, 3>& Source,
        const std::array<Point<T, 3>, 2>& ears) const;

    Vector<T, 3> scale_ellipsoid(const Vector<T, 3>& dir,
                                 double a,
                                 double b,
                                 double c) const;

   private:
    T ArcLengthOnEllipsoid(const Vector<T, 3>& dir_from_center_1,
                           const Vector<T, 3>& dir_from_center_2,
                           ArcMode mode = ArcMode::Accurate,
                           int steps = 100) const;

    T AccurateArcLength(const Vector<T, 3>& unit_start,
                        const Vector<T, 3>& unit_end,
                        double a,
                        double b,
                        double c,
                        int steps) const;

    Vector<T, 3> dEllipsoid_dt(const Vector<T, 3>& unit_start,
                               const Vector<T, 3>& unit_end,
                               double t,
                               double a,
                               double b,
                               double c,
                               double h = 1e-5) const;

   private:
    std::array<T, 3> m_dimensions;
    // Center
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
Ellipsoid<T>::~Ellipsoid() {}

template <typename T>
const std::array<T, 3>& Ellipsoid<T>::GetDimensions() const {
    return m_dimensions;
};

template <typename T>
std::array<T, 2> Ellipsoid<T>::GetIntersection(const Ray<T, 3>& ray) {
    // ray and ellipsoid intersection

    auto direction = ray.GetDirection();

    // This splitting is really inefficient
    T vx = direction[RTB::Axes::x];
    T vy = direction[RTB::Axes::y];
    T vz = direction[RTB::Axes::z];

    auto origin = ray.GetOrigin();

    T x0 = origin[RTB::Axes::x];
    T y0 = origin[RTB::Axes::y];
    T z0 = origin[RTB::Axes::z];

    T A = m_dimensions[RTB::Axes::x];
    T B = m_dimensions[RTB::Axes::y];
    T C = m_dimensions[RTB::Axes::z];

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
Vector<T, 3> Ellipsoid<T>::scale_ellipsoid(const Vector<T, 3>& dir,
                                           double a,
                                           double b,
                                           double c) const {
    return Vector<T, 3>{static_cast<T>(a * dir[0]),
                        static_cast<T>(b * dir[1]),
                        static_cast<T>(c * dir[2])};
}

// Derivative of ellipsoid path at point t using central difference
template <typename T>
Vector<T, 3> Ellipsoid<T>::dEllipsoid_dt(const Vector<T, 3>& unit_start,
                                         const Vector<T, 3>& unit_end,
                                         double t,
                                         double a,
                                         double b,
                                         double c,
                                         double h) const {
    Vector<T, 3> pt1 = scale_ellipsoid(
        RTB::slerp<T>(unit_start, unit_end, static_cast<T>(t - h)),
        static_cast<T>(a),
        static_cast<T>(b),
        static_cast<T>(c));
    Vector<T, 3> pt2 = scale_ellipsoid(
        RTB::slerp<T>(unit_start, unit_end, static_cast<T>(t + h)),
        static_cast<T>(a),
        static_cast<T>(b),
        static_cast<T>(c));
    return Vector<T, 3>{static_cast<T>((pt2[0] - pt1[0]) / (2 * h)),
                        static_cast<T>((pt2[1] - pt1[1]) / (2 * h)),
                        static_cast<T>((pt2[2] - pt1[2]) / (2 * h))};
}
// Arc length via Simpson's rule
template <typename T>
T Ellipsoid<T>::AccurateArcLength(const Vector<T, 3>& unit_start,
                                  const Vector<T, 3>& unit_end,
                                  double a,
                                  double b,
                                  double c,
                                  int steps) const {
    double total = 0.0;
    double h = 1.0 / steps;

    for (int i = 0; i <= steps; ++i) {
        double t = i * h;
        double weight = (i == 0 || i == steps) ? 1 : (i % 2 == 0 ? 2 : 4);
        Vector<T, 3> deriv = dEllipsoid_dt(unit_start, unit_end, t, a, b, c);
        total += weight * deriv.Magnitude();
    }

    return (h / 3.0) * total;
}

template <typename T>
T Ellipsoid<T>::ArcLengthOnEllipsoid(const Vector<T, 3>& dir_from_center_1,
                                     const Vector<T, 3>& dir_from_center_2,
                                     ArcMode mode,
                                     int steps) const {
    Vector<T, 3> u = dir_from_center_1.Normalize();
    Vector<T, 3> v = dir_from_center_2.Normalize();

    if (mode == ArcMode::FastEstimate) {
        double angle = acos(DotProduct(u, v));  // radians
        double r_avg =
            std::cbrt(m_dimensions[0] * m_dimensions[1] * m_dimensions[2]);
        return r_avg * angle;
    } else {
        return AccurateArcLength(
            u, v, m_dimensions[0], m_dimensions[1], m_dimensions[2], steps);
    }
}

template <typename T>
trace_results<T> Ellipsoid<T>::TracePathEllipsoid(
    const Point<T, 3>& Source, const std::array<Point<T, 3>, 2>& ears) const {
    std::array<Point<T, 3>, 2> tangent_points;
    std::array<T, 2> pathlengths;
    std::array<T, 2> arclengths;

    // Construct the polar plane from the source point and ellipsoid
    T a1 = Source[0] / (m_dimensions[0] * m_dimensions[0]);
    T b1 = Source[1] / (m_dimensions[1] * m_dimensions[1]);
    T c1 = Source[2] / (m_dimensions[2] * m_dimensions[2]);
    Plane<T> polar_plane(a1, b1, c1, -1);

    Point<T, 3> origin = {0, 0, 0};

    // Split for ease of use, can be better
    T a = m_dimensions[0];
    T b = m_dimensions[1];
    T c = m_dimensions[2];

    for (size_t Currentear = 0; Currentear < 2; Currentear++) {
        Point<T, 3> ear = ears[Currentear];

        // Construct the plane defined by source, origin, and ear
        Plane<T> STO_plane(origin, ear, Source);

        Ray<T, 3> intersection_line = IntersectPlanes(polar_plane, STO_plane);

        Vector<T, 3> N = intersection_line.GetDirection();
        Point<T, 3> P = intersection_line.GetOrigin();

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

        Point<T, 3> tangent_one = intersection_line.GetPosition(t_plus);
        Point<T, 3> tangent_two = intersection_line.GetPosition(t_minus);

        // Calculate the arc angles between the ear and both tangents
        Vector<T, 3> vecEar(origin, ear);
        // vecEar = unit_vector(vecEar);

        Vector<T, 3> tangent_one_vec(origin, tangent_one);
        // tangent_one_vec = unit_vector(tangent_one_vec);

        Vector<T, 3> tangent_two_vec(origin, tangent_two);
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
        Point<T, 3> closest_tangent;
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

    Vector<T, 3> rayleft(Source, tangent_points[0]);
    Vector<T, 3> rayright(Source, tangent_points[1]);

    std::array<Vector<T, 3>, 2> rays = {rayleft, rayright};

    return {pathlengths, arclengths, tangent_points, rays};
}

}  // namespace RTB

#endif /* ELLIPSOID_HPP */
