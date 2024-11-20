#ifndef STANDARDS_HPP
#define STANDARDS_HPP

// Some stuff you can control for the entire library
namespace RTB {

using RESOLUTION = float;

// Slightly unnecessary - use cmath or math.h instead?
constexpr RESOLUTION PI = 3.141592653589793238462643383279502884;
constexpr RESOLUTION TAU = 2 * PI;
constexpr RESOLUTION PI_2 = PI / 2;
constexpr RESOLUTION PI_4 = PI / 4;
constexpr RESOLUTION PI_180 = PI / 180;
constexpr RESOLUTION R180_PI = 180 / PI;

}  // namespace RTB
#endif  // STANDARDS_HPP