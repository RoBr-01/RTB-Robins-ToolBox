#ifndef MATH_HPP
#define MATH_HPP

// STL
#include <cmath>

// RTB
#include "Standards.hpp"
#include "Vector.hpp"

namespace RTB {

// ==============================
// Coordinate types
// ==============================

/**
 * @brief Cartesian 3D coordinate (x, y, z).
 */
template <typename T>
struct CartesianCoord {
    T x{}, y{}, z{};
};

/**
 * @brief Spherical coordinate (azimuth, elevation, radius).
 *
 * Azimuth and elevation are in degrees. Radius is in the same unit as
 * the originating Cartesian coordinate.
 */
template <typename T>
struct SphericalCoord {
    T az{}, el{}, r{};
};

// ==============================
// Degree/radian conversion
// ==============================

template <typename T>
T deg2rad(T degrees) {
    return degrees * static_cast<T>(PI_div_180);
}

template <typename T>
T rad2deg(T radians) {
    return radians * static_cast<T>(R180_div_PI);
}

// ==============================
// Degree-domain trig
// (all cast through double for precision)
// ==============================

template <typename T>
T sind(T degrees) {
    return static_cast<T>(std::sin(static_cast<double>(degrees) *
                                   static_cast<double>(PI_div_180)));
}

template <typename T>
T asind(T value) {
    return static_cast<T>(std::asin(static_cast<double>(value)) *
                          static_cast<double>(R180_div_PI));
}

template <typename T>
T cosd(T degrees) {
    return static_cast<T>(std::cos(static_cast<double>(degrees) *
                                   static_cast<double>(PI_div_180)));
}

template <typename T>
T acosd(T value) {
    return static_cast<T>(std::acos(static_cast<double>(value)) *
                          static_cast<double>(R180_div_PI));
}

template <typename T>
T tand(T degrees) {
    return static_cast<T>(std::tan(static_cast<double>(degrees) *
                                   static_cast<double>(PI_div_180)));
}

template <typename T>
T atand(T value) {
    return static_cast<T>(std::atan(static_cast<double>(value)) *
                          static_cast<double>(R180_div_PI));
}

// ==============================
// Coordinate conversion
// ==============================

/**
 * @brief Converts Cartesian (x, y, z) to Spherical (az, el, r).
 *
 * Azimuth and elevation are returned in degrees.
 */
template <typename T, typename C>
std::array<T, 3> Cart2Sph(const C& c) {
    const double x = static_cast<double>(c[0]);
    const double y = static_cast<double>(c[1]);
    const double z = static_cast<double>(c[2]);

    const double rxy = std::sqrt(x * x + y * y);

    return {
        static_cast<T>(std::atan2(y, x) * static_cast<double>(R180_div_PI)),
        static_cast<T>(std::atan2(z, rxy) * static_cast<double>(R180_div_PI)),
        static_cast<T>(std::sqrt(x * x + y * y + z * z))};
}

/**
 * @brief Converts Spherical (az, el, r) to Cartesian (x, y, z).
 *
 * Azimuth and elevation are expected in degrees.
 */
template <typename T, typename S>
std::array<T, 3> Sph2Cart(const S& s) {
    const T az = static_cast<T>(s[0]);
    const T el = static_cast<T>(s[1]);
    const T r = static_cast<T>(s[2]);

    const T cosEl = cosd(el);

    return {r * cosEl * cosd(az), r * cosEl * sind(az), r * sind(el)};
}

// ==============================
// dB utilities
// ==============================

/** @brief Amplitude ratio to dB (20 * log10). */
template <typename T>
T frac_to_dB(T frac) {
    return static_cast<T>(20.0 * std::log10(static_cast<double>(frac)));
}

/** @brief dB to amplitude ratio (inverse of 20 * log10). */
template <typename T>
T dB_to_frac(T dB) {
    return static_cast<T>(std::pow(10.0, static_cast<double>(dB) / 20.0));
}

// ==============================
// Polar pattern
// ==============================

/**
 * @brief Evaluates a cardioid-family polar pattern.
 *
 * Returns a signed fractional gain value. The sign indicates polarity:
 * positive values are in the main lobe, negative values are in the rear lobe.
 * To convert to dB: frac_to_dB(std::abs(PolarFrac(...)))
 *
 * @param polar_pattern           Pattern shape factor [0..1]:
 *                                0 = omnidirectional, 1 = figure-of-eight,
 *                                0.5 = cardioid.
 * @param max_attenuation_offset  Offset to prevent log(0) at full null
 *                                (dB floor control). Applied before
 *                                normalization.
 * @param rayvector               Unit vector toward the source (must be
 * normalized).
 * @param zeroaxis                Unit vector of the on-axis direction (must be
 * normalized).
 * @return Signed fractional gain. Magnitude in [0..1], sign encodes polarity.
 */
template <typename T>
T PolarFrac(T polar_pattern,
            T max_attenuation_offset,
            const Vector<T, 3>& rayvector,
            const Vector<T, 3>& zeroaxis) {
    const T alpha = acosd(DotProduct(rayvector, zeroaxis));
    const T signed_frac = polar_pattern + (1 - polar_pattern) * cosd(alpha);
    return (std::abs(signed_frac) + max_attenuation_offset) /
           (1 + max_attenuation_offset);
}

// ==============================
// Spherical linear interpolation
// ==============================

/**
 * @brief Spherical linear interpolation between two unit vectors.
 *
 * @param u  Start unit vector (must be normalized).
 * @param v  End unit vector (must be normalized).
 * @param t  Interpolation parameter [0..1].
 */
template <typename T>
Vector<T, 3> Slerp(const Vector<T, 3>& u, const Vector<T, 3>& v, T t) {
    const T omega = std::acos(static_cast<double>(DotProduct(u, v)));
    if (omega < static_cast<T>(1e-6))
        return u;
    const T sin_omega = std::sin(static_cast<double>(omega));
    const T s1 = std::sin(static_cast<double>((1 - t) * omega)) / sin_omega;
    const T s2 = std::sin(static_cast<double>(t * omega)) / sin_omega;
    return Vector<T, 3>{
        s1 * u[0] + s2 * v[0], s1 * u[1] + s2 * v[1], s1 * u[2] + s2 * v[2]};
}

}  // namespace RTB

#endif  // MATH_HPP