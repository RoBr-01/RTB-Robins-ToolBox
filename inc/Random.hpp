#ifndef RANDOM_HPP
#define RANDOM_HPP

// STL
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <random>
#include <vector>

// RTB
#include "Standards.hpp"
#include "Vector.hpp"

namespace RTB {

namespace Random {

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
Vec3R RandSampleSphere(std::mt19937& generator) {
    std::uniform_real_distribution<RESOLUTION> uniform01(0.0, 1.0);

    RESOLUTION theta = 2 * Math::PI * uniform01(generator);
    RESOLUTION phi = acos(1 - 2 * uniform01(generator));
    RESOLUTION x = sin(phi) * cos(theta);
    RESOLUTION y = sin(phi) * sin(theta);
    RESOLUTION z = cos(phi);

    return Vec3R({x, y, z});
}
}  // namespace Random

}  // namespace RTB
#endif  // RANDOM_HPP