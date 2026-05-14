// GOOGLETEST
#include <gtest/gtest.h>

// STL
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <random>

// LOCAL
#include "RTB/Math.hpp"

namespace {

constexpr double k_abs_tol = 1e-12;
constexpr double k_rel_tol = 1e-12;

bool near(double value_a,
          double value_b,
          double abs_tol = k_abs_tol,
          double rel_tol = k_rel_tol) {
    const double diff = std::abs(value_a - value_b);
    if (diff <= abs_tol) {
        return true;
    }
    return diff <= rel_tol * std::max(std::abs(value_a), std::abs(value_b));
}

void expectNear(double actual,
                double expected,
                double abs_tol = k_abs_tol,
                double rel_tol = k_rel_tol) {
    EXPECT_TRUE(near(actual, expected, abs_tol, rel_tol))
        << "actual=" << actual << ", expected=" << expected;
}

void expectVecNear(const std::array<double, 3>& actual,
                   const std::array<double, 3>& expected,
                   double abs_tol = k_abs_tol,
                   double rel_tol = k_rel_tol) {
    for (std::size_t i = 0; i < 3; ++i) {
        expectNear(actual[i], expected[i], abs_tol, rel_tol);
    }
}

// Normalize angle to [-180, 180)
double normalizeDeg(double deg) {
    double remainder = std::fmod(deg, 360.0);
    if (remainder < -180.0) {
        remainder += 360.0;
    }
    if (remainder >= 180.0) {
        remainder -= 360.0;
    }
    return remainder;
}

void expectAngleNear(double actual, double expected, double tol = 1e-10) {
    const double diff = normalizeDeg(actual - expected);
    EXPECT_NEAR(diff, 0.0, tol)
        << "actual=" << actual << ", expected=" << expected;
}

}  // namespace

// -----------------------------------------------------------------------------
// Degree/radian conversion
// -----------------------------------------------------------------------------

TEST(Math, Deg2RadKnownValues) {
    expectNear(RTB::deg2rad(0.0), 0.0);
    expectNear(RTB::deg2rad(180.0), std::acos(-1.0));
    expectNear(RTB::deg2rad(90.0), std::acos(-1.0) / 2.0);
    expectNear(RTB::deg2rad(-45.0), -std::acos(-1.0) / 4.0);
}

TEST(Math, Rad2DegKnownValues) {
    expectNear(RTB::rad2deg(0.0), 0.0);
    expectNear(RTB::rad2deg(RTB::pi), 180.0);
    expectNear(RTB::rad2deg(RTB::pi / 2.0), 90.0);
    expectNear(RTB::rad2deg(-RTB::pi / 4.0), -45.0);
}

TEST(Math, DegRadRoundTrip) {
    for (double deg = -10000.0; deg <= 10000.0; deg += 0.125) {  // NOLINT
        expectNear(RTB::rad2deg(RTB::deg2rad(deg)), deg);
    }
}

// -----------------------------------------------------------------------------
// Trigonometric functions in degrees
// -----------------------------------------------------------------------------

TEST(Math, SindKnownValues) {
    expectNear(RTB::sind(0.0), 0.0);
    expectNear(RTB::sind(30.0), 0.5);
    expectNear(RTB::sind(90.0), 1.0);
    expectNear(RTB::sind(-90.0), -1.0);
    expectNear(RTB::sind(180.0), 0.0);
}

TEST(Math, CosdKnownValues) {
    expectNear(RTB::cosd(0.0), 1.0);
    expectNear(RTB::cosd(60.0), 0.5);
    expectNear(RTB::cosd(90.0), 0.0);
    expectNear(RTB::cosd(180.0), -1.0);
}

TEST(Math, TandKnownValues) {
    expectNear(RTB::tand(0.0), 0.0);
    expectNear(RTB::tand(45.0), 1.0);
    expectNear(RTB::tand(-45.0), -1.0);
}

TEST(Math, TrigIdentities) {
    for (double deg = -720.0; deg <= 720.0; deg += 0.25) {  // NOLINT
        const double angle_sin = RTB::sind(deg);
        const double angle_cos = RTB::cosd(deg);

        expectNear((angle_sin * angle_sin) + (angle_cos * angle_cos),
                   1.0,
                   1e-12,
                   1e-12);
    }
}

TEST(Math, InverseTrigRoundTrip) {
    for (double value = -1.0; value <= 1.0; value += 0.001) {  // NOLINT
        expectNear(RTB::sind(RTB::asind(value)), value, 1e-12, 1e-12);
        expectNear(RTB::cosd(RTB::acosd(value)), value, 1e-12, 1e-12);
    }

    for (double value = -1000.0; value <= 1000.0; value += 0.25) {  // NOLINT
        expectNear(RTB::tand(RTB::atand(value)), value, 1e-10, 1e-10);
    }
}

// -----------------------------------------------------------------------------
// Coordinate conversion according to SOFA convention
// -----------------------------------------------------------------------------

TEST(Math, Sph2CartPrincipalAxes) {
    // Front (+X)
    expectVecNear(RTB::sph2cart<double>(std::array<double, 3>{0.0, 0.0, 1.0}),
                  {1.0, 0.0, 0.0});

    // Left (+Y)
    expectVecNear(RTB::sph2cart<double>(std::array<double, 3>{90.0, 0.0, 1.0}),
                  {0.0, 1.0, 0.0});

    // Back (-X)
    expectVecNear(RTB::sph2cart<double>(std::array<double, 3>{180.0, 0.0, 1.0}),
                  {-1.0, 0.0, 0.0});

    // Right (-Y)
    expectVecNear(RTB::sph2cart<double>(std::array<double, 3>{-90.0, 0.0, 1.0}),
                  {0.0, -1.0, 0.0});

    // Up (+Z)
    expectVecNear(RTB::sph2cart<double>(std::array<double, 3>{0.0, 90.0, 1.0}),
                  {0.0, 0.0, 1.0});

    // Down (-Z)
    expectVecNear(RTB::sph2cart<double>(std::array<double, 3>{0.0, -90.0, 1.0}),
                  {0.0, 0.0, -1.0});
}

TEST(Math, Cart2SphPrincipalAxes) {
    {
        auto sph = RTB::cart2sph<double>(std::array<double, 3>{1.0, 0.0, 0.0});
        expectAngleNear(sph[0], 0.0);
        expectAngleNear(sph[1], 0.0);
        expectNear(sph[2], 1.0);
    }

    {
        auto sph = RTB::cart2sph<double>(std::array<double, 3>{0.0, 1.0, 0.0});
        expectAngleNear(sph[0], 90.0);
        expectAngleNear(sph[1], 0.0);
        expectNear(sph[2], 1.0);
    }

    {
        auto sph = RTB::cart2sph<double>(std::array<double, 3>{-1.0, 0.0, 0.0});
        expectAngleNear(sph[0], 180.0);
        expectAngleNear(sph[1], 0.0);
        expectNear(sph[2], 1.0);
    }

    {
        auto sph = RTB::cart2sph<double>(std::array<double, 3>{0.0, -1.0, 0.0});
        expectAngleNear(sph[0], -90.0);
        expectAngleNear(sph[1], 0.0);
        expectNear(sph[2], 1.0);
    }

    {
        auto sph = RTB::cart2sph<double>(std::array<double, 3>{0.0, 0.0, 1.0});
        expectAngleNear(sph[0], 0.0);  // atan2(0,0) -> 0 by convention
        expectAngleNear(sph[1], 90.0);
        expectNear(sph[2], 1.0);
    }

    {
        auto sph = RTB::cart2sph<double>(std::array<double, 3>{0.0, 0.0, -1.0});
        expectAngleNear(sph[0], 0.0);
        expectAngleNear(sph[1], -90.0);
        expectNear(sph[2], 1.0);
    }
}

TEST(Math, SphCartRoundTrip) {
    for (double az = -180.0; az <= 180.0; az += 5.0) {    // NOLINT
        for (double el = -90.0; el <= 90.0; el += 5.0) {  // NOLINT
            for (double r = 0.0; r <= 10.0; r += 0.5) {   // NOLINT
                const std::array<double, 3> sph{az, el, r};

                const auto cart = RTB::sph2cart<double>(sph);
                const auto sph2 = RTB::cart2sph<double>(cart);

                expectNear(sph2[2], r, 1e-10, 1e-10);

                if (r == 0.0) {
                    continue;  // angles undefined at origin
                }

                // At poles azimuth is mathematically undefined.
                if (std::abs(std::abs(el) - 90.0) < 1e-12) {
                    expectAngleNear(sph2[1], el);
                } else {
                    expectAngleNear(sph2[0], az);
                    expectAngleNear(sph2[1], el);
                }
            }
        }
    }
}

TEST(Math, CartSphRoundTripRandomized) {
    std::mt19937 rng(0x5EED1234U);  // NOLINT
    std::uniform_real_distribution<double> dist(-1.0e6, 1.0e6);

    for (int i = 0; i < 10000; ++i) {
        const std::array<double, 3> cart{dist(rng), dist(rng), dist(rng)};

        const auto sph = RTB::cart2sph<double>(cart);
        const auto cart2 = RTB::sph2cart<double>(sph);

        expectVecNear(cart2, cart, 1e-8, 1e-10);
    }
}

TEST(Math, Cart2SphOrigin) {
    const auto sph =
        RTB::cart2sph<double>(std::array<double, 3>{0.0, 0.0, 0.0});

    expectNear(sph[0], 0.0);
    expectNear(sph[1], 0.0);
    expectNear(sph[2], 0.0);
}

// -----------------------------------------------------------------------------
// dB conversions
// -----------------------------------------------------------------------------

TEST(Math, Frac2dBKnownValues) {
    expectNear(RTB::frac2dB(1.0), 0.0);
    expectNear(RTB::frac2dB(10.0), 20.0);
    expectNear(RTB::frac2dB(0.1), -20.0);
    expectNear(RTB::frac2dB(2.0), 20.0 * std::log10(2.0));
}

TEST(Math, dB2FracKnownValues) {
    expectNear(RTB::dB2frac(0.0), 1.0);
    expectNear(RTB::dB2frac(20.0), 10.0);
    expectNear(RTB::dB2frac(-20.0), 0.1);
}

TEST(Math, dBRoundTrip) {
    for (double frac = 1e-12; frac <= 1e12; frac *= 1.25) {  // NOLINT
        expectNear(RTB::dB2frac(RTB::frac2dB(frac)), frac, 1e-10, 1e-10);
    }

    for (double db = -300.0; db <= 300.0; db += 0.25) {  // NOLINT
        expectNear(RTB::frac2dB(RTB::dB2frac(db)), db, 1e-10, 1e-10);
    }
}

// -----------------------------------------------------------------------------
// RTB::nextPowerOf2
// -----------------------------------------------------------------------------

TEST(Math, NextPowerOf2KnownValues) {
    EXPECT_EQ(RTB::nextPowerOf2(0U), 1U);
    EXPECT_EQ(RTB::nextPowerOf2(1U), 1U);
    EXPECT_EQ(RTB::nextPowerOf2(2U), 2U);
    EXPECT_EQ(RTB::nextPowerOf2(3U), 4U);
    EXPECT_EQ(RTB::nextPowerOf2(4U), 4U);
    EXPECT_EQ(RTB::nextPowerOf2(5U), 8U);
    EXPECT_EQ(RTB::nextPowerOf2(15U), 16U);
    EXPECT_EQ(RTB::nextPowerOf2(16U), 16U);
    EXPECT_EQ(RTB::nextPowerOf2(17U), 32U);
}

TEST(Math, NextPowerOf2ExhaustiveSmallRange) {
    for (std::size_t number = 0; number < 1'000'000; ++number) {
        const std::size_t power = RTB::nextPowerOf2(number);

        // power is power of two
        EXPECT_EQ((power & (power - 1)), 0U);

        // power covers number
        EXPECT_GE(power, number);

        // power is minimal
        if (power > 1) {
            EXPECT_LT(power >> 1, number);
        }
    }
}