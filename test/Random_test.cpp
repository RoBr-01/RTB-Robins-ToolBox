// GOOGLETEST
#include <gtest/gtest.h>

// STL
#include <cstddef>
#include <cstdint>
#include <random>
#include <type_traits>

// LOCAL
#include <RTB/Random.hpp>
#include <RTB/Vector.hpp>
#include "RTB/Math.hpp"

namespace {

constexpr double epsilon = 1e-9;
constexpr float float_epsilon = 1e-6F;

// ------------------------------------------------------------
// randomValue(uint32_t&)
// ------------------------------------------------------------

TEST(RandomTest, RandomValueReturnsValueInUnitInterval) {
    uint32_t state = 123456789U;

    for (int i = 0; i < 1000; ++i) {
        const float value = RTB::randomValue(state);
        EXPECT_GE(value, 0.0F);
        EXPECT_LE(value, 1.0F);
    }
}

TEST(RandomTest, RandomValueAdvancesState) {
    uint32_t state = 42U;
    const uint32_t original_state = state;

    [[maybe_unused]] const float value = RTB::randomValue(state);

    EXPECT_NE(state, original_state);
}

TEST(RandomTest, RandomValueIsDeterministicForSameSeed) {
    uint32_t state1 = 12345U;
    uint32_t state2 = 12345U;

    for (int i = 0; i < 100; ++i) {
        EXPECT_FLOAT_EQ(RTB::randomValue(state1), RTB::randomValue(state2));
        EXPECT_EQ(state1, state2);
    }
}

TEST(RandomTest, RandomValueDifferentSequentialCallsUsuallyDiffer) {
    uint32_t state = 98765U;

    const float random_a = RTB::randomValue(state);
    const float random_b = RTB::randomValue(state);

    EXPECT_NE(random_a, random_b);
}

// ------------------------------------------------------------
// randomHashValue(uint32_t)
// ------------------------------------------------------------

TEST(RandomTest, RandomHashValueReturnsValueInUnitInterval) {
    for (uint32_t i = 0; i < 1000; ++i) {
        const float value = RTB::randomHashValue(i);
        EXPECT_GE(value, 0.0F);
        EXPECT_LE(value, 1.0F);
    }
}

TEST(RandomTest, RandomHashValueIsDeterministic) {
    for (uint32_t i = 0; i < 1000; ++i) {
        EXPECT_FLOAT_EQ(RTB::randomHashValue(i), RTB::randomHashValue(i));
    }
}

TEST(RandomTest, RandomHashValueDifferentInputsUsuallyDiffer) {
    const float random_a = RTB::randomHashValue(123U);
    const float random_b = RTB::randomHashValue(124U);

    EXPECT_NE(random_a, random_b);
}

// ------------------------------------------------------------
// randSampleSphere<T>()
// ------------------------------------------------------------

TEST(RandomTest, RandSampleSphereReturnsUnitVector) {
    std::mt19937 generator(12345U);  // NOLINT

    for (int i = 0; i < 1000; ++i) {
        auto vector = RTB::randSampleSphere<double>(generator);
        EXPECT_NEAR(vector.magnitude(), 1.0, epsilon);
    }
}

TEST(RandomTest, RandSampleSphereComponentsWithinBounds) {
    std::mt19937 generator(42U);  // NOLINT

    for (int i = 0; i < 1000; ++i) {
        auto vector = RTB::randSampleSphere<double>(generator);

        for (size_t j = 0; j < 3; ++j) {
            EXPECT_GE(vector[j], -1.0);
            EXPECT_LE(vector[j], 1.0);
        }
    }
}

TEST(RandomTest, RandSampleSphereIsDeterministicForSameSeed) {
    std::mt19937 gen1(12345U);  // NOLINT
    std::mt19937 gen2(12345U);  // NOLINT

    for (int i = 0; i < 100; ++i) {
        auto v1 = RTB::randSampleSphere<double>(gen1);
        auto v2 = RTB::randSampleSphere<double>(gen2);

        for (size_t j = 0; j < 3; ++j) {
            EXPECT_NEAR(v1[j], v2[j], epsilon);
        }
    }
}

TEST(RandomTest, RandSampleSphereSupportsFloat) {
    std::mt19937 generator(123U);  // NOLINT

    auto vector = RTB::randSampleSphere<float>(generator);

    static_assert(std::is_same_v<decltype(vector), RTB::Vector<float, 3>>);
    EXPECT_NEAR(vector.magnitude(), 1.0F, float_epsilon);
}

// ------------------------------------------------------------
// randSampleSpherePoint<T>()
// ------------------------------------------------------------

TEST(RandomTest, RandSampleSpherePointReturnsRadiusOne) {
    std::mt19937 generator(12345U);  // NOLINT

    for (int i = 0; i < 1000; ++i) {
        auto point = RTB::randSampleSpherePoint<double>(generator);
        EXPECT_DOUBLE_EQ(point.radius, 1.0);
    }
}

TEST(RandomTest, RandSampleSpherePointAzimuthInRange) {
    std::mt19937 generator(42U);  // NOLINT

    for (int i = 0; i < 1000; ++i) {
        auto point = RTB::randSampleSpherePoint<double>(generator);

        EXPECT_GE(point.azimuth, 0.0);
        EXPECT_LE(point.azimuth, 360.0);
    }
}

TEST(RandomTest, RandSampleSpherePointElevationInRange) {
    std::mt19937 generator(42U);  // NOLINT

    for (int i = 0; i < 1000; ++i) {
        auto point = RTB::randSampleSpherePoint<double>(generator);

        EXPECT_GE(point.elevation, -90.0);
        EXPECT_LE(point.elevation, 90.0);
    }
}

TEST(RandomTest, RandSampleSpherePointIsDeterministicForSameSeed) {
    std::mt19937 gen1(12345U);  // NOLINT
    std::mt19937 gen2(12345U);  // NOLINT

    for (int i = 0; i < 100; ++i) {
        auto p1 = RTB::randSampleSpherePoint<double>(gen1);
        auto p2 = RTB::randSampleSpherePoint<double>(gen2);

        EXPECT_NEAR(p1.azimuth, p2.azimuth, epsilon);
        EXPECT_NEAR(p1.elevation, p2.elevation, epsilon);
        EXPECT_NEAR(p1.radius, p2.radius, epsilon);
    }
}

TEST(RandomTest, RandSampleSpherePointSupportsFloat) {
    std::mt19937 generator(123U);  // NOLINT

    auto point = RTB::randSampleSpherePoint<float>(generator);

    static_assert(std::is_same_v<decltype(point), RTB::SphericalCoord<float>>);
    EXPECT_NEAR(point.radius, 1.0F, float_epsilon);
}

}  // namespace