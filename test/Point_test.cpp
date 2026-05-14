// GOOGLETEST
#include <gtest/gtest.h>

// STL
#include <array>
#include <cmath>
#include <cstddef>
#include <type_traits>
#include <utility>

// LOCAL
#include <RTB/Point.hpp>

namespace {

using Point2d = RTB::Point<double, 2>;
using Point3d = RTB::Point<double, 3>;
using Point3i = RTB::Point<int, 3>;

constexpr double epsilon = 1e-9;

// ------------------------------------------------------------
// Helper Functions
// ------------------------------------------------------------

template <typename PointType>
void expectPointNear(const PointType& actual,
                     const PointType& expected,
                     double eps = epsilon) {
    constexpr std::size_t num_elements =
        std::tuple_size_v<decltype(actual.getCoords())>;
    for (std::size_t i = 0; i < num_elements; ++i) {
        EXPECT_NEAR(actual[i], expected[i], eps) << "Mismatch at index " << i;
    }
}

// ------------------------------------------------------------
// Constructor Tests
// ------------------------------------------------------------

TEST(PointTest, DefaultConstructorInitializesToZero) {
    Point3d point;

    EXPECT_DOUBLE_EQ(point[0], 0.0);
    EXPECT_DOUBLE_EQ(point[1], 0.0);
    EXPECT_DOUBLE_EQ(point[2], 0.0);
}

TEST(PointTest, InitializerListConstructorStoresValues) {
    Point3d point{1.5, -2.0, 7.25};

    EXPECT_DOUBLE_EQ(point[0], 1.5);
    EXPECT_DOUBLE_EQ(point[1], -2.0);
    EXPECT_DOUBLE_EQ(point[2], 7.25);
}

TEST(PointTest, InitializerListConstructorWorksForIntegers) {
    Point3i point{1, 2, 3};

    EXPECT_EQ(point[0], 1);
    EXPECT_EQ(point[1], 2);
    EXPECT_EQ(point[2], 3);
}

// ------------------------------------------------------------
// getCoords()
// ------------------------------------------------------------

TEST(PointTest, GetCoordsReturnsCorrectArray) {
    const Point3d point{1.0, 2.0, 3.0};

    std::array<double, 3> coords = point.getCoords();

    EXPECT_DOUBLE_EQ(coords[0], 1.0);
    EXPECT_DOUBLE_EQ(coords[1], 2.0);
    EXPECT_DOUBLE_EQ(coords[2], 3.0);
}

// ------------------------------------------------------------
// setCoords()
// ------------------------------------------------------------

TEST(PointTest, SetCoordsReplacesCoordinates) {
    Point3d point;

    point.setCoords({4.0, 5.0, 6.0});

    EXPECT_DOUBLE_EQ(point[0], 4.0);
    EXPECT_DOUBLE_EQ(point[1], 5.0);
    EXPECT_DOUBLE_EQ(point[2], 6.0);
}

// ------------------------------------------------------------
// operator[] non-const
// ------------------------------------------------------------

TEST(PointTest, SubscriptOperatorAllowsModification) {
    Point3d point;

    point[0] = 10.0;
    point[1] = 20.0;
    point[2] = 30.0;

    EXPECT_DOUBLE_EQ(point[0], 10.0);
    EXPECT_DOUBLE_EQ(point[1], 20.0);
    EXPECT_DOUBLE_EQ(point[2], 30.0);
}

// ------------------------------------------------------------
// operator[] const
// ------------------------------------------------------------

TEST(PointTest, ConstSubscriptOperatorReturnsCorrectValues) {
    const Point3d point{7.0, 8.0, 9.0};

    EXPECT_DOUBLE_EQ(point[0], 7.0);
    EXPECT_DOUBLE_EQ(point[1], 8.0);
    EXPECT_DOUBLE_EQ(point[2], 9.0);
}

// ------------------------------------------------------------
// distance2Points()
// ------------------------------------------------------------

TEST(PointTest, DistanceBetweenIdenticalPointsIsZero) {
    const Point3d point{1.0, 2.0, 3.0};

    const double distance = RTB::distance2Points(point, point);

    EXPECT_NEAR(distance, 0.0, epsilon);
}

TEST(PointTest, DistanceInTwoDimensions) {
    const Point2d point_1{0.0, 0.0};
    const Point2d point_2{3.0, 4.0};

    const double distance = RTB::distance2Points(point_1, point_2);

    EXPECT_NEAR(distance, 5.0, epsilon);
}

TEST(PointTest, DistanceInThreeDimensions) {
    const Point3d point_1{1.0, 2.0, 3.0};
    const Point3d point_2{4.0, 6.0, 3.0};

    const double distance = RTB::distance2Points(point_1, point_2);

    EXPECT_NEAR(distance, 5.0, epsilon);
}

TEST(PointTest, DistanceIsSymmetric) {
    const Point3d point_1{1.0, -2.0, 3.0};
    const Point3d point_2{-4.0, 5.0, 0.5};

    const double d1 = RTB::distance2Points(point_1, point_2);
    const double d2 = RTB::distance2Points(point_2, point_1);

    EXPECT_NEAR(d1, d2, epsilon);
}

TEST(PointTest, DistanceSupportsMixedTypes) {
    const RTB::Point<int, 3> point_1{0, 0, 0};
    const RTB::Point<double, 3> point_2{1.0, 2.0, 2.0};

    auto distance = RTB::distance2Points(point_1, point_2);

    static_assert(std::is_same_v<decltype(distance), double>);
    EXPECT_NEAR(distance, 3.0, epsilon);
}

// ------------------------------------------------------------
// midpoint2Points()
// ------------------------------------------------------------

TEST(PointTest, MidpointBetweenIdenticalPointsIsSamePoint) {
    const Point3d point{1.0, 2.0, 3.0};

    auto mid = RTB::midpoint2Points(point, point);

    expectPointNear(mid, point);
}

TEST(PointTest, MidpointInTwoDimensions) {
    const Point2d point_1{0.0, 0.0};
    const Point2d point_2{4.0, 6.0};

    auto mid = RTB::midpoint2Points(point_1, point_2);

    EXPECT_DOUBLE_EQ(mid[0], 2.0);
    EXPECT_DOUBLE_EQ(mid[1], 3.0);
}

TEST(PointTest, MidpointInThreeDimensions) {
    const Point3d point_1{1.0, 2.0, 3.0};
    const Point3d point_2{5.0, 6.0, 7.0};

    auto mid = RTB::midpoint2Points(point_1, point_2);

    EXPECT_DOUBLE_EQ(mid[0], 3.0);
    EXPECT_DOUBLE_EQ(mid[1], 4.0);
    EXPECT_DOUBLE_EQ(mid[2], 5.0);
}

TEST(PointTest, MidpointSupportsMixedTypes) {
    const RTB::Point<int, 3> point_1{0, 0, 0};
    const RTB::Point<double, 3> point_2{1.0, 2.0, 3.0};

    auto mid = RTB::midpoint2Points(point_1, point_2);

    static_assert(std::is_same_v<decltype(mid), RTB::Point<double, 3>>);

    EXPECT_DOUBLE_EQ(mid[0], 0.5);
    EXPECT_DOUBLE_EQ(mid[1], 1.0);
    EXPECT_DOUBLE_EQ(mid[2], 1.5);
}

TEST(PointTest, MidpointIsSymmetric) {
    const Point3d point_1{1.0, 2.0, 3.0};
    const Point3d point_2{7.0, 8.0, 9.0};

    auto mid1 = RTB::midpoint2Points(point_1, point_2);
    auto mid2 = RTB::midpoint2Points(point_2, point_1);

    expectPointNear(mid1, mid2);
}

// ------------------------------------------------------------
// Type Trait Tests
// ------------------------------------------------------------

TEST(PointTest, DistanceReturnTypeUsesCommonType) {
    using point_1 = RTB::Point<int, 2>;
    using point_2 = RTB::Point<float, 2>;

    using ReturnType = decltype(RTB::distance2Points(std::declval<point_1>(),
                                                     std::declval<point_2>()));

    static_assert(std::is_same_v<ReturnType, float>);
    SUCCEED();
}

TEST(PointTest, MidpointReturnTypeUsesCommonType) {
    using point_1 = RTB::Point<int, 2>;
    using point_2 = RTB::Point<double, 2>;

    using ReturnType = decltype(RTB::midpoint2Points(std::declval<point_1>(),
                                                     std::declval<point_2>()));

    static_assert(std::is_same_v<ReturnType, RTB::Point<double, 2>>);

    SUCCEED();
}

// ------------------------------------------------------------
// Optional Death Tests (only if assertions are enabled)
// ------------------------------------------------------------

#ifndef NDEBUG

TEST(PointTest, InitializerListWrongSizeTriggersAssert) {
    EXPECT_DEATH((RTB::Point<double, 3>{1.0, 2.0}), "");
}
// NOLINTBEGIN
TEST(PointTest, OutOfBoundsAccessTriggersAssert) {
    EXPECT_DEATH(([]() {
                     RTB::Point<double, 3> point{1.0, 2.0, 3.0};
                     [[maybe_unused]] auto value = point[3];
                 }()),
                 "");
}
// NOLINTEND

#endif  // NDEBUG

// ------------------------------------------------------------
// Additional Coverage Tests
// ------------------------------------------------------------

TEST(PointTest, GetCoordsReturnsCopy) {
    Point3d point{1.0, 2.0, 3.0};

    auto coords = point.getCoords();
    coords[0] = 99.0;
    coords[1] = 88.0;
    coords[2] = 77.0;

    // Original point should remain unchanged.
    EXPECT_DOUBLE_EQ(point[0], 1.0);
    EXPECT_DOUBLE_EQ(point[1], 2.0);
    EXPECT_DOUBLE_EQ(point[2], 3.0);
}

TEST(PointTest, SetCoordsOverwritesExistingValues) {
    Point3d point{1.0, 2.0, 3.0};

    point.setCoords({7.0, 8.0, 9.0});

    EXPECT_DOUBLE_EQ(point[0], 7.0);
    EXPECT_DOUBLE_EQ(point[1], 8.0);
    EXPECT_DOUBLE_EQ(point[2], 9.0);
}

TEST(PointTest, DistanceHandlesNegativeCoordinates) {
    const Point3d point_1{-1.0, -2.0, -3.0};
    const Point3d point_2{2.0, 2.0, 1.0};

    const double distance = RTB::distance2Points(point_1, point_2);

    // Differences: (3, 4, 4)
    // Distance = sqrt(9 + 16 + 16) = sqrt(41)
    EXPECT_NEAR(distance, std::sqrt(41.0), epsilon);
}

TEST(PointTest, MidpointHandlesNegativeCoordinates) {
    const Point3d point_1{-2.0, 4.0, -6.0};
    const Point3d point_2{2.0, 0.0, 2.0};

    auto mid = RTB::midpoint2Points(point_1, point_2);

    EXPECT_DOUBLE_EQ(mid[0], 0.0);
    EXPECT_DOUBLE_EQ(mid[1], 2.0);
    EXPECT_DOUBLE_EQ(mid[2], -2.0);
}

TEST(PointTest, WorksForHigherDimensions) {
    RTB::Point<double, 5> point{1.0, 2.0, 3.0, 4.0, 5.0};

    EXPECT_DOUBLE_EQ(point[0], 1.0);
    EXPECT_DOUBLE_EQ(point[1], 2.0);
    EXPECT_DOUBLE_EQ(point[2], 3.0);
    EXPECT_DOUBLE_EQ(point[3], 4.0);
    EXPECT_DOUBLE_EQ(point[4], 5.0);
}

TEST(PointTest, DistanceWorksForHigherDimensions) {
    const RTB::Point<double, 5> point_1{0.0, 0.0, 0.0, 0.0, 0.0};
    const RTB::Point<double, 5> point_2{1.0, 2.0, 2.0, 1.0, 2.0};

    const double distance = RTB::distance2Points(point_1, point_2);

    // sqrt(1 + 4 + 4 + 1 + 4) = sqrt(14)
    EXPECT_NEAR(distance, std::sqrt(14.0), epsilon);
}

TEST(PointTest, MidpointWorksForHigherDimensions) {
    const RTB::Point<double, 5> point_1{0.0, 2.0, 4.0, 6.0, 8.0};
    const RTB::Point<double, 5> point_2{2.0, 4.0, 6.0, 8.0, 10.0};

    auto mid = RTB::midpoint2Points(point_1, point_2);

    EXPECT_DOUBLE_EQ(mid[0], 1.0);
    EXPECT_DOUBLE_EQ(mid[1], 3.0);
    EXPECT_DOUBLE_EQ(mid[2], 5.0);
    EXPECT_DOUBLE_EQ(mid[3], 7.0);
    EXPECT_DOUBLE_EQ(mid[4], 9.0);
}

}  // namespace