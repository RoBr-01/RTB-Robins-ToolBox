#ifndef STANDARDS_HPP
#define STANDARDS_HPP

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

namespace RTB {

// ==============================
// Mathematical constants
// ==============================

constexpr double PI = 3.141592653589793238462643383279502884;
constexpr double TAU = 2.0 * PI;
constexpr double PI_div_2 = PI / 2.0;
constexpr double PI_div_4 = PI / 4.0;
constexpr double PI_div_180 = PI / 180.0;
constexpr double R180_div_PI = 180.0 / PI;

// ==============================
// Physical constants
// ==============================

/** @brief Speed of sound in dry air at ~20°C (m/s). */
constexpr double SpeedOfSound = 344.0;

/** @brief Speed of light in vacuum (m/s). */
constexpr double SpeedOfLight = 299'792'458.0;

// ==============================
// Enumerations
// ==============================

enum Axes { X, Y, Z };
enum Directions { Left, Right, Front, Back, Up, Down };
enum Vertices { LFD, RFD, LBD, RBD, LFU, RFU, LBU, RBU };

}  // namespace RTB

#endif  // STANDARDS_HPP