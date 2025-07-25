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

constexpr double PI = 3.141592653589793238462643383279502884;
constexpr double TAU = 2 * PI;
constexpr double PI_div_2 = PI / 2;
constexpr double PI_div_4 = PI / 4;
constexpr double PI_div_180 = PI / 180;
constexpr double R180_div_PI = 180 / PI;

enum class Axes { x, y, z };
enum class Coefficients { a, b, c, d };
enum class Directions { Left, Right, Front, Back, Up, Down };
enum class Vertices { LFD, RFD, LBD, RBD, LFU, RFU, LBU, RBU };

constexpr double speed_of_sound = 344;          // meters per second
constexpr double speed_of_light = 299'792'458;  // meters per second

}  // namespace RTB
#endif  // STANDARDS_HPP