#include <gtest/gtest.h>
#include "Vector.hpp"
#include "Point.hpp"

using namespace RTB;

using Vec3 = Vector<double, 3>;
using Point3 = Point<double, 3>;

TEST(VectorTest, DefaultConstructor) {
    Vec3 v;
    for (size_t i = 0; i < 3; ++i)
        EXPECT_EQ(v[i], 0.0);
}

TEST(VectorTest, InitializerListConstructor) {
    Vec3 v{1.0, 2.0, 3.0};
    EXPECT_EQ(v[0], 1.0);
    EXPECT_EQ(v[1], 2.0);
    EXPECT_EQ(v[2], 3.0);
}

TEST(VectorTest, ArrayConstructor) {
    std::array<double, 3> arr = {4.0, 5.0, 6.0};
    Vec3 v(arr);
    EXPECT_EQ(v.GetComponents(), arr);
}

TEST(VectorTest, PointConstructor) {
    Point3 p1{1.0, 2.0, 3.0};
    Vec3 v(p1);
    EXPECT_EQ(v[0], 1.0);
    EXPECT_EQ(v[1], 2.0);
    EXPECT_EQ(v[2], 3.0);
}

TEST(VectorTest, TwoPointConstructor) {
    Point3 p1{1.0, 1.0, 1.0};
    Point3 p2{4.0, 5.0, 6.0};
    Vec3 v(p1, p2);
    EXPECT_EQ(v[0], 3.0);
    EXPECT_EQ(v[1], 4.0);
    EXPECT_EQ(v[2], 5.0);
}

TEST(VectorTest, ArithmeticOperatorsScalar) {
    Vec3 v{1.0, 2.0, 3.0};
    auto v2 = v + 2.0;
    EXPECT_EQ(v2, Vec3({3.0, 4.0, 5.0}));

    v2 = v - 1.0;
    EXPECT_EQ(v2, Vec3({0.0, 1.0, 2.0}));

    v2 = v * 2.0;
    EXPECT_EQ(v2, Vec3({2.0, 4.0, 6.0}));

    v2 = v / 2.0;
    EXPECT_EQ(v2, Vec3({0.5, 1.0, 1.5}));
}

TEST(VectorTest, ArithmeticOperatorsVector) {
    Vec3 v1{1.0, 2.0, 3.0};
    Vec3 v2{4.0, 5.0, 6.0};

    EXPECT_EQ(v1 + v2, Vec3({5.0, 7.0, 9.0}));
    EXPECT_EQ(v2 - v1, Vec3({3.0, 3.0, 3.0}));
    EXPECT_EQ(v1 * v2, Vec3({4.0, 10.0, 18.0}));
}

TEST(VectorTest, CompoundAssignmentOperators) {
    Vec3 v{1.0, 2.0, 3.0};
    v += 1.0;
    EXPECT_EQ(v, Vec3({2.0, 3.0, 4.0}));

    v -= 1.0;
    EXPECT_EQ(v, Vec3({1.0, 2.0, 3.0}));

    v *= 2.0;
    EXPECT_EQ(v, Vec3({2.0, 4.0, 6.0}));

    v /= 2.0;
    EXPECT_EQ(v, Vec3({1.0, 2.0, 3.0}));

    Vec3 other{1.0, 1.0, 1.0};
    v += other;
    EXPECT_EQ(v, Vec3({2.0, 3.0, 4.0}));

    v -= other;
    EXPECT_EQ(v, Vec3({1.0, 2.0, 3.0}));

    v *= other;
    EXPECT_EQ(v, Vec3({1.0, 2.0, 3.0}));
}

TEST(VectorTest, LengthCalculations) {
    Vec3 v{3.0, 4.0, 0.0};
    EXPECT_DOUBLE_EQ(v.Length_squared(), 25.0);
    EXPECT_DOUBLE_EQ(v.Length(), 5.0);
}

TEST(VectorTest, Normalize) {
    Vec3 v{0.0, 3.0, 4.0};
    v.Normalize();
    EXPECT_NEAR(v[0], 0.0, 1e-9);
    EXPECT_NEAR(v[1], 0.6, 1e-9);
    EXPECT_NEAR(v[2], 0.8, 1e-9);
    EXPECT_NEAR(v.Length(), 1.0, 1e-9);
}

TEST(VectorTest, Invert) {
    Vec3 v{1.0, -2.0, 3.0};
    v.Invert();
    EXPECT_EQ(v, Vec3({-1.0, 2.0, -3.0}));
}

TEST(VectorTest, DotProduct) {
    Vec3 v1{1.0, 2.0, 3.0};
    Vec3 v2{4.0, -5.0, 6.0};
    EXPECT_DOUBLE_EQ(dotprod(v1, v2), 12.0);
}

TEST(VectorTest, CrossProduct) {
    Vec3 v1{1.0, 0.0, 0.0};
    Vec3 v2{0.0, 1.0, 0.0};
    auto cross = crossprod(v1, v2);
    EXPECT_EQ(cross, Vec3({0.0, 0.0, 1.0}));
}

TEST(VectorTest, EqualityOperators) {
    Vec3 v1{1.0, 2.0, 3.0};
    Vec3 v2{1.0, 2.0, 3.0};
    Vec3 v3{3.0, 2.0, 1.0};

    EXPECT_TRUE(v1 == v2);
    EXPECT_FALSE(v1 != v2);
    EXPECT_TRUE(v1 != v3);
}