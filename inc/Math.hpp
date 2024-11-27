#ifndef MATH_HPP
#define MATH_HPP

#include <array>
#include <cmath>

#include "inc/Standards.hpp"

// Generally trying to prevent function calls within function calls

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

// The cart2sph and sph2cart follow SOFA convention
// TODO: update to work with any container (of size 3)
template <typename T>
std::array<T, 3> Cart2SphD(const std::array<T, 3> &Cart) {
  std::array<T, 3> Spherical;

  T x = Cart[0];
  T y = Cart[1];
  T z = Cart[2];

  Spherical[0] = (std::atan2(y, x))*R180_PI;  // Azimuth
  Spherical[1] =
      (std::atan2(z, std::sqrt(x * x + y * y))) * R180_PI;  // Elevation
  Spherical[2] = std::sqrt(Cart[0] * Cart[0] + Cart[1] * Cart[1] +
                           Cart[2] * Cart[2]);  // Radius

  return Spherical;
}

template <typename T>
std::array<T, 3> Sph2CartD(const std::array<T, 3> &Spherical) {
  std::array<T, 3> Cartesian;

  T Az = Spherical[0];
  T El = Spherical[1];
  T r = Spherical[2];

  // Since i want to prevent internal function calls, formula again, could be
  // replaced by cosd and sind
  Cartesian[0] = r * std::cos(El * PI_180) * std::cos(Az * PI_180);  // x
  Cartesian[1] = r * std::cos(El * PI_180) * std::sin(Az * PI_180);  // y
  Cartesian[2] = r * std::sin(El * PI_180);                          // z

  return Cartesian;
}

//John Carmacks's fast inverse sqrt, but in c++
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

}  // namespace RTB

#endif  // MATH_HPP