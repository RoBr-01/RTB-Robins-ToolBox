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

/** @brief Speed of sound in dry air at ~20°C (m/s). */
constexpr double speed_of_sound = 344.0;

/** @brief Speed of light in vacuum (m/s). */
constexpr double speed_of_light = 299'792'458.0;

enum Axes { X, Y, Z };
enum Directions { Left, Right, Front, Back, Up, Down };
enum Vertices { LFD, RFD, LBD, RBD, LFU, RFU, LBU, RBU };

}  // namespace RTB

#endif  // STANDARDS_HPP