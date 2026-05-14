#include "RTB/Math.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <random>

using namespace RTB;

namespace {

constexpr double kAbsTol = 1e-12;
constexpr double kRelTol = 1e-12;

bool Near(double a,
          double b,
          double abs_tol = kAbsTol,
          double rel_tol = kRelTol) {
    const double diff = std::abs(a - b);
    if (diff <= abs_tol) {
        return true;
    }
    return diff <= rel_tol * std::max(std::abs(a), std::abs(b));
}

void ExpectNear(double actual,
                double expected,
                double abs_tol = kAbsTol,
                double rel_tol = kRelTol) {
    EXPECT_TRUE(Near(actual, expected, abs_tol, rel_tol))
        << "actual=" << actual << ", expected=" << expected;
}

void ExpectVecNear(const std::array<double, 3>& actual,
                   const std::array<double, 3>& expected,
                   double abs_tol = kAbsTol,
                   double rel_tol = kRelTol) {
    for (std::size_t i = 0; i < 3; ++i) {
        ExpectNear(actual[i], expected[i], abs_tol, rel_tol);
    }
}

// Normalize angle to [-180, 180)
double NormalizeDeg(double deg) {
    double x = std::fmod(deg, 360.0);
    if (x < -180.0) {
        x += 360.0;
    }
    if (x >= 180.0) {
        x -= 360.0;
    }
    return x;
}

void ExpectAngleNear(double actual, double expected, double tol = 1e-10) {
    const double diff = NormalizeDeg(actual - expected);
    EXPECT_NEAR(diff, 0.0, tol)
        << "actual=" << actual << ", expected=" << expected;
}

}  // namespace

// -----------------------------------------------------------------------------
// Degree/radian conversion
// -----------------------------------------------------------------------------

TEST(Math, Deg2RadKnownValues) {
    ExpectNear(deg2rad(0.0), 0.0);
    ExpectNear(deg2rad(180.0), std::acos(-1.0));
    ExpectNear(deg2rad(90.0), std::acos(-1.0) / 2.0);
    ExpectNear(deg2rad(-45.0), -std::acos(-1.0) / 4.0);
}

TEST(Math, Rad2DegKnownValues) {
    ExpectNear(rad2deg(0.0), 0.0);
    ExpectNear(rad2deg(pi), 180.0);
    ExpectNear(rad2deg(pi / 2.0), 90.0);
    ExpectNear(rad2deg(-pi / 4.0), -45.0);
}

TEST(Math, DegRadRoundTrip) {
    for (double deg = -10000.0; deg <= 10000.0; deg += 0.125) {
        ExpectNear(rad2deg(deg2rad(deg)), deg);
    }
}

// -----------------------------------------------------------------------------
// Trigonometric functions in degrees
// -----------------------------------------------------------------------------

TEST(Math, SindKnownValues) {
    ExpectNear(sind(0.0), 0.0);
    ExpectNear(sind(30.0), 0.5);
    ExpectNear(sind(90.0), 1.0);
    ExpectNear(sind(-90.0), -1.0);
    ExpectNear(sind(180.0), 0.0);
}

TEST(Math, CosdKnownValues) {
    ExpectNear(cosd(0.0), 1.0);
    ExpectNear(cosd(60.0), 0.5);
    ExpectNear(cosd(90.0), 0.0);
    ExpectNear(cosd(180.0), -1.0);
}

TEST(Math, TandKnownValues) {
    ExpectNear(tand(0.0), 0.0);
    ExpectNear(tand(45.0), 1.0);
    ExpectNear(tand(-45.0), -1.0);
}

TEST(Math, TrigIdentities) {
    for (double deg = -720.0; deg <= 720.0; deg += 0.25) {
        const double s = sind(deg);
        const double c = cosd(deg);

        ExpectNear((s * s) + (c * c), 1.0, 1e-12, 1e-12);
    }
}

TEST(Math, InverseTrigRoundTrip) {
    for (double x = -1.0; x <= 1.0; x += 0.001) {
        ExpectNear(sind(asind(x)), x, 1e-12, 1e-12);
        ExpectNear(cosd(acosd(x)), x, 1e-12, 1e-12);
    }

    for (double x = -1000.0; x <= 1000.0; x += 0.25) {
        ExpectNear(tand(atand(x)), x, 1e-10, 1e-10);
    }
}

// -----------------------------------------------------------------------------
// Coordinate conversion according to SOFA convention
// -----------------------------------------------------------------------------

TEST(Math, Sph2CartPrincipalAxes) {
    // Front (+X)
    ExpectVecNear(sph2cart<double>(std::array<double, 3>{0.0, 0.0, 1.0}),
                  {1.0, 0.0, 0.0});

    // Left (+Y)
    ExpectVecNear(sph2cart<double>(std::array<double, 3>{90.0, 0.0, 1.0}),
                  {0.0, 1.0, 0.0});

    // Back (-X)
    ExpectVecNear(sph2cart<double>(std::array<double, 3>{180.0, 0.0, 1.0}),
                  {-1.0, 0.0, 0.0});

    // Right (-Y)
    ExpectVecNear(sph2cart<double>(std::array<double, 3>{-90.0, 0.0, 1.0}),
                  {0.0, -1.0, 0.0});

    // Up (+Z)
    ExpectVecNear(sph2cart<double>(std::array<double, 3>{0.0, 90.0, 1.0}),
                  {0.0, 0.0, 1.0});

    // Down (-Z)
    ExpectVecNear(sph2cart<double>(std::array<double, 3>{0.0, -90.0, 1.0}),
                  {0.0, 0.0, -1.0});
}

TEST(Math, Cart2SphPrincipalAxes) {
    {
        auto sph = cart2sph<double>(std::array<double, 3>{1.0, 0.0, 0.0});
        ExpectAngleNear(sph[0], 0.0);
        ExpectAngleNear(sph[1], 0.0);
        ExpectNear(sph[2], 1.0);
    }

    {
        auto sph = cart2sph<double>(std::array<double, 3>{0.0, 1.0, 0.0});
        ExpectAngleNear(sph[0], 90.0);
        ExpectAngleNear(sph[1], 0.0);
        ExpectNear(sph[2], 1.0);
    }

    {
        auto sph = cart2sph<double>(std::array<double, 3>{-1.0, 0.0, 0.0});
        ExpectAngleNear(sph[0], 180.0);
        ExpectAngleNear(sph[1], 0.0);
        ExpectNear(sph[2], 1.0);
    }

    {
        auto sph = cart2sph<double>(std::array<double, 3>{0.0, -1.0, 0.0});
        ExpectAngleNear(sph[0], -90.0);
        ExpectAngleNear(sph[1], 0.0);
        ExpectNear(sph[2], 1.0);
    }

    {
        auto sph = cart2sph<double>(std::array<double, 3>{0.0, 0.0, 1.0});
        ExpectAngleNear(sph[0], 0.0);  // atan2(0,0) -> 0 by convention
        ExpectAngleNear(sph[1], 90.0);
        ExpectNear(sph[2], 1.0);
    }

    {
        auto sph = cart2sph<double>(std::array<double, 3>{0.0, 0.0, -1.0});
        ExpectAngleNear(sph[0], 0.0);
        ExpectAngleNear(sph[1], -90.0);
        ExpectNear(sph[2], 1.0);
    }
}

TEST(Math, SphCartRoundTrip) {
    for (double az = -180.0; az <= 180.0; az += 5.0) {
        for (double el = -90.0; el <= 90.0; el += 5.0) {
            for (double r = 0.0; r <= 10.0; r += 0.5) {
                const std::array<double, 3> sph{az, el, r};

                const auto cart = sph2cart<double>(sph);
                const auto sph2 = cart2sph<double>(cart);

                ExpectNear(sph2[2], r, 1e-10, 1e-10);

                if (r == 0.0) {
                    continue;  // angles undefined at origin
                }

                // At poles azimuth is mathematically undefined.
                if (std::abs(std::abs(el) - 90.0) < 1e-12) {
                    ExpectAngleNear(sph2[1], el);
                } else {
                    ExpectAngleNear(sph2[0], az);
                    ExpectAngleNear(sph2[1], el);
                }
            }
        }
    }
}

TEST(Math, CartSphRoundTripRandomized) {
    std::mt19937 rng(0x5EED1234U);
    std::uniform_real_distribution<double> dist(-1.0e6, 1.0e6);

    for (int i = 0; i < 10000; ++i) {
        const std::array<double, 3> cart{dist(rng), dist(rng), dist(rng)};

        const auto sph = cart2sph<double>(cart);
        const auto cart2 = sph2cart<double>(sph);

        ExpectVecNear(cart2, cart, 1e-8, 1e-10);
    }
}

TEST(Math, Cart2SphOrigin) {
    const auto sph = cart2sph<double>(std::array<double, 3>{0.0, 0.0, 0.0});

    ExpectNear(sph[0], 0.0);
    ExpectNear(sph[1], 0.0);
    ExpectNear(sph[2], 0.0);
}

// -----------------------------------------------------------------------------
// dB conversions
// -----------------------------------------------------------------------------

TEST(Math, Frac2dBKnownValues) {
    ExpectNear(frac2dB(1.0), 0.0);
    ExpectNear(frac2dB(10.0), 20.0);
    ExpectNear(frac2dB(0.1), -20.0);
    ExpectNear(frac2dB(2.0), 20.0 * std::log10(2.0));
}

TEST(Math, dB2FracKnownValues) {
    ExpectNear(dB2frac(0.0), 1.0);
    ExpectNear(dB2frac(20.0), 10.0);
    ExpectNear(dB2frac(-20.0), 0.1);
}

TEST(Math, dBRoundTrip) {
    for (double frac = 1e-12; frac <= 1e12; frac *= 1.25) {
        ExpectNear(dB2frac(frac2dB(frac)), frac, 1e-10, 1e-10);
    }

    for (double db = -300.0; db <= 300.0; db += 0.25) {
        ExpectNear(frac2dB(dB2frac(db)), db, 1e-10, 1e-10);
    }
}

// -----------------------------------------------------------------------------
// nextPowerOf2
// -----------------------------------------------------------------------------

TEST(Math, NextPowerOf2KnownValues) {
    EXPECT_EQ(nextPowerOf2(0U), 1U);
    EXPECT_EQ(nextPowerOf2(1U), 1U);
    EXPECT_EQ(nextPowerOf2(2U), 2U);
    EXPECT_EQ(nextPowerOf2(3U), 4U);
    EXPECT_EQ(nextPowerOf2(4U), 4U);
    EXPECT_EQ(nextPowerOf2(5U), 8U);
    EXPECT_EQ(nextPowerOf2(15U), 16U);
    EXPECT_EQ(nextPowerOf2(16U), 16U);
    EXPECT_EQ(nextPowerOf2(17U), 32U);
}

TEST(Math, NextPowerOf2ExhaustiveSmallRange) {
    for (std::size_t n = 0; n < 1'000'000; ++n) {
        const std::size_t p = nextPowerOf2(n);

        // p is power of two
        EXPECT_EQ((p & (p - 1)), 0U);

        // p covers n
        EXPECT_GE(p, n);

        // p is minimal
        if (p > 1) {
            EXPECT_LT(p >> 1, n);
        }
    }
}