#ifndef MATH_HPP
#define MATH_HPP

// STL
#include <array>
#include <cmath>

// RTB
#include "Standards.hpp"
#include "Vector.hpp"

namespace RTB {

template <typename T>
T deg2rad(const T &degrees) {
    return degrees * PI_180;
}

template <typename T>
T rad2deg(const T &radians) {
    return radians * R180_PI;
}

template <typename T>
T sind(const T &degrees) {
    return std::sin(degrees * PI_180);
}

template <typename T>
T asind(const T &value) {
    return std::asin(value) * R180_PI;
}

template <typename T>
T cosd(const T &degrees) {
    return std::cos(degrees * PI_180);
}

template <typename T>
T acosd(const T &value) {
    return std::acos(value) * R180_PI;
}

template <typename T>
T tand(const T &degrees) {
    return std::tan(degrees * PI_180);
}

template <typename T>
T atand(const T &value) {
    return std::atan(value) * R180_PI;
}

template <typename T>
std::array<T, 3> Cart2SphD(const std::array<T, 3> &cartesian) {
    std::array<T, 3> spherical;

    T x = cartesian[0];
    T y = cartesian[1];
    T z = cartesian[2];

    spherical[0] = (std::atan2(y, x))*R180_PI;  // Azimuth
    spherical[1] =
        (std::atan2(z, std::sqrt(x * x + y * y))) * R180_PI;  // Elevation
    spherical[2] = std::sqrt(x * x + y * y + z * z);          // Radius

    return spherical;
}

template <typename T>
std::array<T, 3> Sph2CartD(const std::array<T, 3> &Spherical) {
    std::array<T, 3> Cartesian;

    T Az = Spherical[0];
    T El = Spherical[1];
    T r = Spherical[2];

    T cosEl = cosd(El);
    T sinEl = sind(El);
    T cosAz = cosd(Az);
    T sinAz = sind(Az);

    Cartesian[0] = r * cosEl * cosAz;  // x
    Cartesian[1] = r * cosEl * sinAz;  // y
    Cartesian[2] = r * sinEl;          // z

    return Cartesian;
}

// John Carmacks's fast inverse sqrt, but in c++
template <typename T>
T fastInverseSqrt(T number) {
    constexpr T threeHalves = static_cast<T>(1.5);

    T x2 = number * static_cast<T>(0.5);
    T y = number;

    // Use different magic numbers depending on the type
    std::uint64_t i;
    if constexpr (std::is_same<T, double>::value) {
        i = *reinterpret_cast<std::uint64_t *>(&y);  // Double magic constant
        i = 0x5FE6EB50C7B537A9 - (i >> 1);           // Magic number for double
    } else {
        i = *reinterpret_cast<std::uint32_t *>(&y);  // Float magic constant
        i = 0x5f3759df - (i >> 1);                   // Magic number for float
    }

    y = *reinterpret_cast<T *>(&i);  // Convert back to floating-point

    y = y * (threeHalves - (x2 * y * y));  // One iteration of Newton's method
    // y = y * (threeHalves - (x2 * y * y));  // Second iteration - optional

    return y;
}

template <typename T>
T frac_to_dB(T Frac) {
    return 20 * std::log10(Frac);
}

template <typename T>
T dB_to_frac(T dB) {
    return std::pow(10, (dB / 20));
}

// TODO: split into positive and negative part of curve,
// use negative part to invert polarity of signal
// Split the absolute value
template <typename T>
T PolardB(T polar_pattern,
          T max_attenuation_offset,
          Vec3R rayvector,
          Vec3R zeroaxis) {
    T alpha = acosd(dotprod(rayvector, zeroaxis));

    T frac = (std::abs(polar_pattern + (1 - polar_pattern) * cosd(alpha)) +
              max_attenuation_offset) /
             (1 + max_attenuation_offset);
    T dB = frac_to_dB(frac);

    return dB;
}

}  // namespace RTB

#endif  // MATH_HPP