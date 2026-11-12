/*
 ____ _____ ____
|  _ \_   _| __ )
| |_) || | |  _ \
|  _ < | | | |_) |
|_| \_\|_| |____/

 Random.hpp
 Pseudo-random and hash-based sampling helpers (uniform, sphere sampling).

 Copyright (c) 2026 Robin Breugelmans
 Licensed under the MIT License. See LICENSE in the project root.
*/

#ifndef RTB_RANDOM_HPP
#define RTB_RANDOM_HPP

// STL
#include <cmath>
#include <cstdint>
#include <random>

// RTB
#include <RTB/Math.hpp>
#include <RTB/Vector.hpp>

namespace RTB {

// ==============================
// PCG (Permuted Congruential Generator)
// Thanks to: www.pcg-random.org and www.shadertoy.com/view/XlGch
// ==============================

/**
 * @brief Returns a pseudo-random float in [0, 1] and advances the state.
 *
 * Uses a PCG generator — good statistical quality and cheap to compute.
 * Suitable for sequential sampling in hot paths.
 */
inline float randomValue(uint32_t& state) {
    state = (state * 747796405U) + 2891336453U;
    uint32_t result = ((state >> ((state >> 28U) + 4U)) ^ state) * 277803737U;
    result = (result >> 22U) ^ result;
    return static_cast<float>(result) / 4294967295.0F;
}

/**
 * @brief Returns a pseudo-random float in [0, 1] from a stateless hash.
 *
 * WARNING: Low statistical quality — produces visible patterns for structured
 * or sequential inputs. Use RandomValue() with a persistent state for anything
 * requiring good distribution. Only suitable for non-critical one-off hashing.
 */
inline float randomHashValue(uint32_t state) {
    state *= (state + 195439U) * (state + 124395U) * (state + 845921U);
    return static_cast<float>(state) / 4294967295.0F;
}

// ==============================
// Sphere sampling
// ==============================

/**
 * @brief Returns a uniformly distributed random unit vector on the sphere.
 *
 * Uses the Marsaglia / spherical coordinates method.
 * The generator is advanced by two samples per call.
 */
template <typename T>
Vector<T, 3> randSampleSphere(std::mt19937& generator) {
    static std::uniform_real_distribution<T> uniform01(static_cast<T>(0),
                                                       static_cast<T>(1));

    const T theta =
        static_cast<T>(2) * static_cast<T>(pi) * uniform01(generator);
    const T phi =
        std::acos(static_cast<T>(1) - static_cast<T>(2) * uniform01(generator));
    const T sinphi = std::sin(phi);

    return Vector<T, 3>{
        sinphi * std::cos(theta), sinphi * std::sin(theta), std::cos(phi),
    };
}

/**
 * @brief Returns a uniformly distributed random point on the unit sphere
 *        as a SphericalCoord (az, el, r=1) in degrees.
 *
 * The generator is advanced by two samples per call.
 */
template <typename T>
SphericalCoord<T> randSampleSpherePoint(std::mt19937& generator) {
    static std::uniform_real_distribution<T> uniform01(static_cast<T>(0),
                                                       static_cast<T>(1));

    const T theta =
        static_cast<T>(2) * static_cast<T>(pi) * uniform01(generator);
    const T phi =
        std::acos(static_cast<T>(2) * uniform01(generator) - static_cast<T>(1));

    return SphericalCoord<T>{
        rad2deg(theta), static_cast<T>(90) - rad2deg(phi), static_cast<T>(1),
    };
}

}  // namespace RTB

#endif /* RTB_RANDOM_HPP */