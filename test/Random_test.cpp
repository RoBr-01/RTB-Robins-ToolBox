// Random_test.cpp
//
// GoogleTest suite for RTB/Random.hpp
//
// Covered functionality:
// - randomValue(uint32_t&)
// - randomHashValue(uint32_t)
// - randSampleSphere<T>(std::mt19937&)
// - randSampleSpherePoint<T>(std::mt19937&)

#include <gtest/gtest.h>

#include <RTB/Random.hpp>
#include <RTB/Vector.hpp>
#include <cstddef>
#include <cstdint>
#include <random>
#include <type_traits>

#include "RTB/Math.hpp"

namespace {

constexpr double kEps = 1e-9;
constexpr float kFloatEps = 1e-6F;

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
    const uint32_t originalState = state;

    [[maybe_unused]] const float value = RTB::randomValue(state);

    EXPECT_NE(state, originalState);
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

    const float a = RTB::randomValue(state);
    const float b = RTB::randomValue(state);

    EXPECT_NE(a, b);
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
    const float a = RTB::randomHashValue(123U);
    const float b = RTB::randomHashValue(124U);

    EXPECT_NE(a, b);
}

// ------------------------------------------------------------
// randSampleSphere<T>()
// ------------------------------------------------------------

TEST(RandomTest, RandSampleSphereReturnsUnitVector) {
    std::mt19937 generator(12345U);

    for (int i = 0; i < 1000; ++i) {
        auto v = RTB::randSampleSphere<double>(generator);
        EXPECT_NEAR(v.magnitude(), 1.0, kEps);
    }
}

TEST(RandomTest, RandSampleSphereComponentsWithinBounds) {
    std::mt19937 generator(42U);

    for (int i = 0; i < 1000; ++i) {
        auto v = RTB::randSampleSphere<double>(generator);

        for (size_t j = 0; j < 3; ++j) {
            EXPECT_GE(v[j], -1.0);
            EXPECT_LE(v[j], 1.0);
        }
    }
}

TEST(RandomTest, RandSampleSphereIsDeterministicForSameSeed) {
    std::mt19937 gen1(12345U);
    std::mt19937 gen2(12345U);

    for (int i = 0; i < 100; ++i) {
        auto v1 = RTB::randSampleSphere<double>(gen1);
        auto v2 = RTB::randSampleSphere<double>(gen2);

        for (size_t j = 0; j < 3; ++j) {
            EXPECT_NEAR(v1[j], v2[j], kEps);
        }
    }
}

TEST(RandomTest, RandSampleSphereSupportsFloat) {
    std::mt19937 generator(123U);

    auto v = RTB::randSampleSphere<float>(generator);

    static_assert(std::is_same_v<decltype(v), RTB::Vector<float, 3>>);
    EXPECT_NEAR(v.magnitude(), 1.0F, kFloatEps);
}

// ------------------------------------------------------------
// randSampleSpherePoint<T>()
// ------------------------------------------------------------

TEST(RandomTest, RandSampleSpherePointReturnsRadiusOne) {
    std::mt19937 generator(12345U);

    for (int i = 0; i < 1000; ++i) {
        auto point = RTB::randSampleSpherePoint<double>(generator);
        EXPECT_DOUBLE_EQ(point.radius, 1.0);
    }
}

TEST(RandomTest, RandSampleSpherePointAzimuthInRange) {
    std::mt19937 generator(42U);

    for (int i = 0; i < 1000; ++i) {
        auto point = RTB::randSampleSpherePoint<double>(generator);

        EXPECT_GE(point.azimuth, 0.0);
        EXPECT_LE(point.azimuth, 360.0);
    }
}

TEST(RandomTest, RandSampleSpherePointElevationInRange) {
    std::mt19937 generator(42U);

    for (int i = 0; i < 1000; ++i) {
        auto point = RTB::randSampleSpherePoint<double>(generator);

        EXPECT_GE(point.elevation, -90.0);
        EXPECT_LE(point.elevation, 90.0);
    }
}

TEST(RandomTest, RandSampleSpherePointIsDeterministicForSameSeed) {
    std::mt19937 gen1(12345U);
    std::mt19937 gen2(12345U);

    for (int i = 0; i < 100; ++i) {
        auto p1 = RTB::randSampleSpherePoint<double>(gen1);
        auto p2 = RTB::randSampleSpherePoint<double>(gen2);

        EXPECT_NEAR(p1.azimuth, p2.azimuth, kEps);
        EXPECT_NEAR(p1.elevation, p2.elevation, kEps);
        EXPECT_NEAR(p1.radius, p2.radius, kEps);
    }
}

TEST(RandomTest, RandSampleSpherePointSupportsFloat) {
    std::mt19937 generator(123U);

    auto point = RTB::randSampleSpherePoint<float>(generator);

    static_assert(std::is_same_v<decltype(point), RTB::SphericalCoord<float>>);
    EXPECT_NEAR(point.radius, 1.0F, kFloatEps);
}

}  // namespace