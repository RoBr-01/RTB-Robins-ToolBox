#ifndef RANDOM_HPP
#define RANDOM_HPP

// STL
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <random>
#include <vector>

// RTB
#include "Point.hpp"
#include "Standards.hpp"
#include "Vector.hpp"

namespace RTB {

// PCG (permuted congruential generator). Thanks to:
// www.pcg-random.org and www.shadertoy.com/view/XlGch
float RandomValue(uint32_t& state) {
    state = state * 747796405u + 2891336453u;  // Update state
    uint32_t result = ((state >> ((state >> 28) + 4)) ^ state) * 277803737u;
    result = (result >> 22) ^ result;                   // Apply permutation
    return static_cast<float>(result) / 4294967295.0f;  // Normalize to [0, 1]
}

float RandomHashValue(uint32_t state) {
    state *= (state + 195439u) * (state + 124395u) * (state + 845921u);
    return static_cast<float>(state) / 4294967295.0f;  // Normalize to [0, 1]
}

// Returns a random ray
template <typename T>
Vector<T, 3> RandSampleSphere(std::mt19937& generator) {
    std::uniform_real_distribution<T> uniform01(0.0, 1.0);

    T theta = 2 * PI * uniform01(generator);
    T phi = acos(1 - 2 * uniform01(generator));
    T x = sin(phi) * cos(theta);
    T y = sin(phi) * sin(theta);
    T z = cos(phi);

    return Vector<T, 3>({x, y, z});
}

template <typename T>
Point<T, 3> RandSampleSpherePoint(std::mt19937& generator) {
    std::uniform_real_distribution<T> uniform01(0.0, 1.0);
    T theta = 2 * PI * uniform01(generator);
    T phi = acos(2 * uniform01(generator) - 1);

    return {rad2deg(theta), 90 - rad2deg(phi), 1.0f};
}

}  // namespace RTB
#endif /* RANDOM_HPP */
