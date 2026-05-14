#include <gtest/gtest.h>

#include <RTB/Point.hpp>
#include <RTB/Ray.hpp>
#include <RTB/Vector.hpp>
#include <cmath>
#include <type_traits>

namespace {

using Ray3d = RTB::Ray<double, 3>;
using Point3d = RTB::Point<double, 3>;
using Vector3d = RTB::Vector<double, 3>;

constexpr double kEps = 1e-9;

// ------------------------------------------------------------
// Constructor / Accessors
// ------------------------------------------------------------

TEST(RayTest, DefaultConstructorWorks) {
    Ray3d ray;

    // Default constructed Point/Vector should be zero-initialized
    EXPECT_DOUBLE_EQ(ray.getOrigin()[0], 0.0);
    EXPECT_DOUBLE_EQ(ray.getOrigin()[1], 0.0);
    EXPECT_DOUBLE_EQ(ray.getOrigin()[2], 0.0);

    EXPECT_DOUBLE_EQ(ray.getDirection()[0], 0.0);
    EXPECT_DOUBLE_EQ(ray.getDirection()[1], 0.0);
    EXPECT_DOUBLE_EQ(ray.getDirection()[2], 0.0);
}

TEST(RayTest, ConstructorStoresValues) {
    Point3d origin{1.0, 2.0, 3.0};
    Vector3d dir{4.0, 5.0, 6.0};

    Ray3d ray(origin, dir);

    EXPECT_EQ(ray.getOrigin()[0], 1.0);
    EXPECT_EQ(ray.getOrigin()[1], 2.0);
    EXPECT_EQ(ray.getOrigin()[2], 3.0);

    EXPECT_EQ(ray.getDirection()[0], 4.0);
    EXPECT_EQ(ray.getDirection()[1], 5.0);
    EXPECT_EQ(ray.getDirection()[2], 6.0);
}

// ------------------------------------------------------------
// set()
// ------------------------------------------------------------

TEST(RayTest, SetReplacesOriginAndDirection) {
    Ray3d ray;

    Point3d origin{1.0, 2.0, 3.0};
    Vector3d dir{4.0, 5.0, 6.0};

    ray.set(origin, dir);

    EXPECT_EQ(ray.getOrigin()[0], 1.0);
    EXPECT_EQ(ray.getDirection()[0], 4.0);
}

// ------------------------------------------------------------
// normalize()
// ------------------------------------------------------------

TEST(RayTest, NormalizeNormalizesDirection) {
    Ray3d ray(Point3d{0.0, 0.0, 0.0}, Vector3d{3.0, 4.0, 0.0});

    ray.normalize();

    const auto& d = ray.getDirection();

    EXPECT_NEAR(std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]), 1.0, kEps);
}

// ------------------------------------------------------------
// getPosition()
// ------------------------------------------------------------

TEST(RayTest, GetPositionAtZeroReturnsOrigin) {
    Ray3d ray(Point3d{1.0, 2.0, 3.0}, Vector3d{4.0, 5.0, 6.0});

    auto p = ray.getPosition(0.0);

    EXPECT_DOUBLE_EQ(p[0], 1.0);
    EXPECT_DOUBLE_EQ(p[1], 2.0);
    EXPECT_DOUBLE_EQ(p[2], 3.0);
}

TEST(RayTest, GetPositionLinearInterpolation) {
    Ray3d ray(Point3d{1.0, 2.0, 3.0}, Vector3d{1.0, 0.0, -1.0});

    auto p = ray.getPosition(2.0);

    // origin + 2 * direction = (3, 2, 1)
    EXPECT_DOUBLE_EQ(p[0], 3.0);
    EXPECT_DOUBLE_EQ(p[1], 2.0);
    EXPECT_DOUBLE_EQ(p[2], 1.0);
}

TEST(RayTest, GetPositionNegativeStepMovesBackward) {
    Ray3d ray(Point3d{0.0, 0.0, 0.0}, Vector3d{1.0, 0.0, 0.0});

    auto p = ray.getPosition(-3.0);

    EXPECT_DOUBLE_EQ(p[0], -3.0);
    EXPECT_DOUBLE_EQ(p[1], 0.0);
    EXPECT_DOUBLE_EQ(p[2], 0.0);
}

// ------------------------------------------------------------
// Invariants
// ------------------------------------------------------------

TEST(RayTest, SetThenGetIsConsistent) {
    Ray3d ray;

    Point3d o{5.0, 6.0, 7.0};
    Vector3d d{1.0, 2.0, 3.0};

    ray.set(o, d);

    EXPECT_EQ(ray.getOrigin()[0], o[0]);
    EXPECT_EQ(ray.getDirection()[1], d[1]);
}

}  // namespace