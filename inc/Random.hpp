// PCG (permuted congruential generator). Thanks to:
// www.pcg-random.org and www.shadertoy.com/view/XlGch
#include <cstdint> // For uint32_t

float RandomValue(uint32_t& state) {
    state = state * 747796405u + 2891336453u; // Update state
    uint32_t result = ((state >> ((state >> 28) + 4)) ^ state) * 277803737u; 
    result = (result >> 22) ^ result; // Apply permutation
    return static_cast<float>(result) / 4294967295.0f; // Normalize to [0, 1]
}

float RandomHashValue(uint32_t state) {
    state *= (state + 195439u) * (state + 124395u) * (state + 845921u); 
    return static_cast<float>(state) / 4294967295.0f; // Normalize to [0, 1]
}