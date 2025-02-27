#ifndef STANDARDS_HPP
#define STANDARDS_HPP

#include <type_traits>

/*
 * Axes follow the SOFA convention:
 * https://www.sofaconventions.org/mediawiki/index.php/SOFA_specifications
 * Cartesian is thus basically the physics convention (right hand rule):
 *   FRONT = +X, BACK  = -X
 *   LEFT  = +Y, RIGHT = -Y
 *   UP    = +Z, DOWN  = -Z
 * Spherical:
 *   +Azimuth = rotate from +X to +Y
 *   -Azimuth = rotate from +X to -Y
 *   +Elevation = rotate from XY-Plane to +Z
 *   -Elevation = rotate from XY-Plane to -Z
 */

// Some stuff you can control for the entire library
namespace RTB {

using RESOLUTION = double;

constexpr RESOLUTION Epsilon = std::numeric_limits<RESOLUTION>::epsilon();

// auto Test = make_value(10);
// std::cout << typeid(Test).name() << std::endl;
// Explicit constructor for a standard-complient type
template <typename T>
inline RESOLUTION make_value(T value) {
    return static_cast<RESOLUTION>(value);
}

namespace Math {

constexpr RESOLUTION PI = 3.141592653589793238462643383279502884;
constexpr RESOLUTION TAU = 2 * PI;
constexpr RESOLUTION PI_2 = PI / 2;
constexpr RESOLUTION PI_4 = PI / 4;
constexpr RESOLUTION PI_180 = PI / 180;
constexpr RESOLUTION R180_PI = 180 / PI;

}  // namespace Math

namespace  Orientation{

enum class Axes { x, y, z };
enum class Coefficients { a, b, c, d };
enum class Directions { Left, Right, Front, Back, Up, Down };
enum class Vertices { LFD, RFD, LBD, RBD, LFU, RFU, LBU, RBU };

}  // namespace enums

template <typename T>
constexpr auto to_index(T e) noexcept {
    return static_cast<std::underlying_type_t<T>>(e);
}

namespace Physics {

constexpr RESOLUTION SpeedOfSound = 344;    // meters per second
constexpr RESOLUTION ReferenceVolume = 85;  // dB SPL @ 1m
}  // namespace physics

}  // namespace RTB
#endif  // STANDARDS_HPP