#ifndef MATH_HPP
#define MATH_HPP

// STL
#include <array>
#include <cmath>
#include <cstddef>

namespace RTB {

/**
 * @brief Math constants
 */
constexpr double pi = 3.141592653589793238462643383279502884;
constexpr double tau = 2.0 * pi;
constexpr double pi_div_2 = pi / 2.0;
constexpr double pi_div_4 = pi / 4.0;
constexpr double pi_div_180 = pi / 180.0;
constexpr double r180_div_pi = 180.0 / pi;

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
    T azimuth{}, elevation{}, radius{};
};

template <typename T>
[[nodiscard]] T deg2rad(T degrees) {
    return degrees * static_cast<T>(pi_div_180);
}

template <typename T>
[[nodiscard]] T rad2deg(T radians) {
    return radians * static_cast<T>(r180_div_pi);
}

template <typename T>
[[nodiscard]] T sind(T degrees) {
    return static_cast<T>(std::sin(static_cast<double>(degrees) *
                                   static_cast<double>(pi_div_180)));
}

template <typename T>
[[nodiscard]] T asind(T value) {
    return static_cast<T>(std::asin(static_cast<double>(value)) *
                          static_cast<double>(r180_div_pi));
}

template <typename T>
[[nodiscard]] T cosd(T degrees) {
    return static_cast<T>(std::cos(static_cast<double>(degrees) *
                                   static_cast<double>(pi_div_180)));
}

template <typename T>
[[nodiscard]] T acosd(T value) {
    return static_cast<T>(std::acos(static_cast<double>(value)) *
                          static_cast<double>(r180_div_pi));
}

template <typename T>
[[nodiscard]] T tand(T degrees) {
    return static_cast<T>(std::tan(static_cast<double>(degrees) *
                                   static_cast<double>(pi_div_180)));
}

template <typename T>
[[nodiscard]] T atand(T value) {
    return static_cast<T>(std::atan(static_cast<double>(value)) *
                          static_cast<double>(r180_div_pi));
}

/**
 * @brief Converts Cartesian (x, y, z) to Spherical (azimuth, elevation,
 * radius).
 *
 * Azimuth and elevation are returned in degrees.
 */
template <typename T, typename C>
[[nodiscard]] std::array<T, 3> cart2sph(const C& cart) {
    const T cart_x = static_cast<T>(cart[0]);
    const T cart_y = static_cast<T>(cart[1]);
    const T cart_z = static_cast<T>(cart[2]);

    const T rxy = std::sqrt((cart_x * cart_x) + (cart_y * cart_y));

    return {
        static_cast<T>(std::atan2(cart_y, cart_x) *
                       static_cast<T>(r180_div_pi)),
        static_cast<T>(std::atan2(cart_z, rxy) * static_cast<T>(r180_div_pi)),
        static_cast<T>(std::sqrt((cart_x * cart_x) + (cart_y * cart_y) +
                                 (cart_z * cart_z)))};
}

/**
 * @brief Converts Spherical (azimuth, elevation, radius) to Cartesian (x, y,
 * z).
 *
 * Azimuth and elevation are expected in degrees.
 */
template <typename T, typename S>
[[nodiscard]] std::array<T, 3> sph2cart(const S& sph) {
    const T azimuth = static_cast<T>(sph[0]);
    const T elevation = static_cast<T>(sph[1]);
    const T radius = static_cast<T>(sph[2]);

    const T cos_el = cosd(elevation);

    return {radius * cos_el * cosd(azimuth),
            radius * cos_el * sind(azimuth),
            radius * sind(elevation)};
}

/** @brief Amplitude ratio to dB (20 * log10). */
template <typename T>
[[nodiscard]] T frac2dB(T frac) {
    return static_cast<T>(20.0 * std::log10(static_cast<double>(frac)));
}

/** @brief dB to amplitude ratio (inverse of 20 * log10). */
template <typename T>
[[nodiscard]] T dB2frac(T decibel) {
    return static_cast<T>(std::pow(10.0, static_cast<double>(decibel) / 20.0));
}

// TODO: expand to be a template?
[[nodiscard]] inline size_t nextPowerOf2(const size_t number) {
    if (number == 0) {
        return 1;
    }
    size_t power = 1;
    while (power < number) {
        power <<= 1;
    }
    return power;
}

}  // namespace RTB

#endif /* MATH_HPP */
