#ifndef STANDARDS_HPP
#define STANDARDS_HPP

/*
Axes follow the SOFA convention:
https://www.sofaconventions.org/mediawiki/index.php/SOFA_specifications
Cartesian is thus basically the physics convention (right hand rule)
FRONT = +X, BACK = -X
LEFT = +Y, RIGHT = -Y
UP = +Z, DOWN = -Z
Spherical just has the Elevation reversed
+Azimuth = rotate from +X to +Y
-Azimuth = rotate from +X to -Y
+Elevation = rotate from XY-Plane to +Z
-Elevation = rotate from XY-Plane to -Z
*/

// Some stuff you can control for the entire library
namespace RTB {

using RESOLUTION = float;

// auto Test = make_value(10);
// std::cout << typeid(Test).name() << std::endl;
template <typename T>
inline RESOLUTION make_value(T value) {
    return static_cast<RESOLUTION>(value);
}

// Slightly unnecessary - use cmath or math.h instead?
constexpr RESOLUTION PI = 3.141592653589793238462643383279502884;
constexpr RESOLUTION TAU = 2 * PI;
constexpr RESOLUTION PI_2 = PI / 2;
constexpr RESOLUTION PI_4 = PI / 4;
constexpr RESOLUTION PI_180 = PI / 180;
constexpr RESOLUTION R180_PI = 180 / PI;

// Optional but useful:
enum Axes { x, y, z };
enum Coefficients { a, b, c, d };
enum Directions { Left, Right, Front, Back, Up, Down };
enum Vertices { LFD, RFD, LBD, RBD, LFU, RFU, LBU, RBU };

RESOLUTION SpeedOfSound = 344;    // meters per second
RESOLUTION ReferenceVolume = 85;  // dB SPL @ 1m

}  // namespace RTB
#endif  // STANDARDS_HPP