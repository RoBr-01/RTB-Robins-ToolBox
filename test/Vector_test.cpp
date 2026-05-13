#include <gtest/gtest.h>

#include <RTB/RTB.hpp>
#include <cmath>
#include <cstddef>
#include <random>

using namespace RTB;

using Vec3 = Vector<double, 3>;
using Point3 = Point<double, 3>;

namespace {

constexpr double kTol = 1e-12;

void ExpectVecNear(const Vec3& actual,
                   const Vec3& expected,
                   double tol = kTol) {
    for (std::size_t i = 0; i < 3; ++i) {
        EXPECT_NEAR(actual[i], expected[i], tol) << "Mismatch at index " << i;
    }
}

}  // namespace

// -----------------------------------------------------------------------------
// Constructors
// -----------------------------------------------------------------------------

TEST(VectorTest, DefaultConstructor) {
    Vec3 v;
    for (std::size_t i = 0; i < 3; ++i) {
        EXPECT_EQ(v[i], 0.0);
    }
}

TEST(VectorTest, InitializerListConstructor) {
    Vec3 v{1.0, 2.0, 3.0};

    EXPECT_EQ(v[0], 1.0);
    EXPECT_EQ(v[1], 2.0);
    EXPECT_EQ(v[2], 3.0);
}

TEST(VectorTest, ArrayConstructor) {
    std::array<double, 3> arr{4.0, 5.0, 6.0};
    Vec3 v(arr);

    EXPECT_EQ(v.getComponents(), arr);
}

TEST(VectorTest, PointConstructor) {
    Point3 p{1.0, 2.0, 3.0};
    Vec3 v(p);

    EXPECT_EQ(v, Vec3({1.0, 2.0, 3.0}));
}

TEST(VectorTest, TwoPointConstructor) {
    Point3 p1{1.0, 1.0, 1.0};
    Point3 p2{4.0, 5.0, 6.0};

    Vec3 v(p1, p2);

    EXPECT_EQ(v, Vec3({3.0, 4.0, 5.0}));
}

// -----------------------------------------------------------------------------
// Basic utilities
// -----------------------------------------------------------------------------

TEST(VectorTest, Size) {
    EXPECT_EQ(Vec3::size(), 3u);
}

TEST(VectorTest, SetAndGetComponents) {
    Vec3 v;
    std::array<double, 3> components{7.0, 8.0, 9.0};

    v.setComponents(components);

    EXPECT_EQ(v.getComponents(), components);
}

TEST(VectorTest, IndexOperator) {
    Vec3 v{1.0, 2.0, 3.0};

    EXPECT_EQ(v[0], 1.0);
    EXPECT_EQ(v[1], 2.0);
    EXPECT_EQ(v[2], 3.0);

    v[1] = 10.0;
    EXPECT_EQ(v[1], 10.0);
}

// -----------------------------------------------------------------------------
// Scalar arithmetic
// -----------------------------------------------------------------------------

TEST(VectorTest, ArithmeticOperatorsScalar) {
    Vec3 v{1.0, 2.0, 3.0};

    EXPECT_EQ(v + 2.0, Vec3({3.0, 4.0, 5.0}));
    EXPECT_EQ(v - 1.0, Vec3({0.0, 1.0, 2.0}));
    EXPECT_EQ(v * 2.0, Vec3({2.0, 4.0, 6.0}));
    EXPECT_EQ(v / 2.0, Vec3({0.5, 1.0, 1.5}));
}

TEST(VectorTest, ScalarOperatorsCommutative) {
    Vec3 v{1.0, 2.0, 3.0};

    EXPECT_EQ(2.0 + v, Vec3({3.0, 4.0, 5.0}));
    EXPECT_EQ(v + 2.0, 2.0 + v);

    EXPECT_EQ(3.0 * v, Vec3({3.0, 6.0, 9.0}));
    EXPECT_EQ(v * 3.0, 3.0 * v);

    EXPECT_EQ(5.0 - v, Vec3({4.0, 3.0, 2.0}));
}

// -----------------------------------------------------------------------------
// Vector arithmetic
// -----------------------------------------------------------------------------

TEST(VectorTest, ArithmeticOperatorsVector) {
    Vec3 v1{1.0, 2.0, 3.0};
    Vec3 v2{4.0, 5.0, 6.0};

    EXPECT_EQ(v1 + v2, Vec3({5.0, 7.0, 9.0}));
    EXPECT_EQ(v2 - v1, Vec3({3.0, 3.0, 3.0}));
    EXPECT_EQ(v1 * v2, Vec3({4.0, 10.0, 18.0}));
}

TEST(VectorTest, ElementWiseDivision) {
    Vec3 a{6.0, 8.0, 10.0};
    Vec3 b{2.0, 4.0, 5.0};

    EXPECT_EQ(a / b, Vec3({3.0, 2.0, 2.0}));

    a /= b;
    EXPECT_EQ(a, Vec3({3.0, 2.0, 2.0}));
}

// -----------------------------------------------------------------------------
// Compound assignment
// -----------------------------------------------------------------------------

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

    v /= other;
    EXPECT_EQ(v, Vec3({1.0, 2.0, 3.0}));
}

// -----------------------------------------------------------------------------
// Unary operations
// -----------------------------------------------------------------------------

TEST(VectorTest, UnaryMinus) {
    Vec3 v{1.0, -2.0, 3.0};

    Vec3 negated = -v;

    EXPECT_EQ(negated, Vec3({-1.0, 2.0, -3.0}));
    EXPECT_EQ(v, Vec3({1.0, -2.0, 3.0}));
}

TEST(VectorTest, Invert) {
    Vec3 v{1.0, -2.0, 3.0};

    Vec3& result = v.invert();

    EXPECT_EQ(v, Vec3({-1.0, 2.0, -3.0}));
    EXPECT_EQ(&result, &v);
}

// -----------------------------------------------------------------------------
// Magnitude and normalization
// -----------------------------------------------------------------------------

TEST(VectorTest, LengthCalculations) {
    Vec3 v{3.0, 4.0, 0.0};

    EXPECT_DOUBLE_EQ(v.magnitudeSquared(), 25.0);
    EXPECT_DOUBLE_EQ(v.magnitude(), 5.0);
}

TEST(VectorTest, Normalize) {
    Vec3 v{0.0, 3.0, 4.0};
    Vec3 original = v;

    Vec3 normalized = v.normalize();

    ExpectVecNear(normalized, Vec3({0.0, 0.6, 0.8}));
    EXPECT_NEAR(normalized.magnitude(), 1.0, kTol);

    EXPECT_EQ(v, original);
}

TEST(VectorTest, NormalizeInPlace) {
    Vec3 v{0.0, 3.0, 4.0};

    Vec3& result = v.normalizeInPlace();

    ExpectVecNear(v, Vec3({0.0, 0.6, 0.8}));
    EXPECT_NEAR(v.magnitude(), 1.0, kTol);
    EXPECT_EQ(&result, &v);
}

TEST(VectorTest, NormalizeZeroVector) {
    Vec3 zero{0.0, 0.0, 0.0};

    EXPECT_EQ(zero.normalize(), zero);

    zero.normalizeInPlace();
    EXPECT_EQ(zero, Vec3({0.0, 0.0, 0.0}));
}

TEST(VectorTest, ChainOperations) {
    Vec3 v{0.0, 6.0, 8.0};

    v.normalizeInPlace().invert();

    ExpectVecNear(v, Vec3({0.0, -0.6, -0.8}));
}

// -----------------------------------------------------------------------------
// Dot product
// -----------------------------------------------------------------------------

TEST(VectorTest, DotProduct) {
    Vec3 v1{1.0, 2.0, 3.0};
    Vec3 v2{4.0, -5.0, 6.0};

    EXPECT_DOUBLE_EQ(dotProduct(v1, v2), 12.0);
}

TEST(VectorTest, DotProductMixedTypes) {
    Vector<float, 3> vf{1.0f, 2.0f, 3.0f};
    Vector<double, 3> vd{4.0, -5.0, 6.0};

    auto result = dotProduct(vf, vd);

    EXPECT_DOUBLE_EQ(result, 12.0);
}

TEST(VectorTest, DotProductDistributive) {
    Vec3 a{1.0, 2.0, 3.0};
    Vec3 b{4.0, 5.0, 6.0};
    Vec3 c{7.0, 8.0, 9.0};

    EXPECT_NEAR(
        dotProduct(a, b + c), dotProduct(a, b) + dotProduct(a, c), kTol);
}

// -----------------------------------------------------------------------------
// Cross product
// -----------------------------------------------------------------------------

TEST(VectorTest, CrossProduct) {
    Vec3 v1{1.0, 0.0, 0.0};
    Vec3 v2{0.0, 1.0, 0.0};

    EXPECT_EQ(crossProduct(v1, v2), Vec3({0.0, 0.0, 1.0}));
    EXPECT_EQ(crossProduct(v2, v1), Vec3({0.0, 0.0, -1.0}));
}

TEST(VectorTest, CrossProductMixedTypes) {
    Vector<float, 3> vf{1.0f, 0.0f, 0.0f};
    Vector<double, 3> vd{0.0, 1.0, 0.0};

    EXPECT_EQ(crossProduct(vf, vd), Vec3({0.0, 0.0, 1.0}));
}

TEST(VectorTest, CrossProductOrthogonality) {
    Vec3 a{1.0, 2.0, 3.0};
    Vec3 b{4.0, 5.0, 6.0};

    Vec3 c = crossProduct(a, b);

    EXPECT_NEAR(dotProduct(a, c), 0.0, kTol);
    EXPECT_NEAR(dotProduct(b, c), 0.0, kTol);
}

TEST(VectorTest, CrossProductLagrangeIdentity) {
    Vec3 a{1.0, 2.0, 3.0};
    Vec3 b{4.0, 5.0, 6.0};

    Vec3 c = crossProduct(a, b);

    const double lhs = c.magnitudeSquared();
    const double rhs = a.magnitudeSquared() * b.magnitudeSquared() -
                       std::pow(dotProduct(a, b), 2.0);

    EXPECT_NEAR(lhs, rhs, kTol);
}

// -----------------------------------------------------------------------------
// Equality
// -----------------------------------------------------------------------------

TEST(VectorTest, EqualityOperators) {
    Vec3 v1{1.0, 2.0, 3.0};
    Vec3 v2{1.0, 2.0, 3.0};
    Vec3 v3{3.0, 2.0, 1.0};

    EXPECT_TRUE(v1 == v2);
    EXPECT_FALSE(v1 != v2);

    EXPECT_TRUE(v1 != v3);
    EXPECT_FALSE(v1 == v3);
}

// -----------------------------------------------------------------------------
// Stream output and print
// -----------------------------------------------------------------------------

TEST(VectorTest, StreamOutput) {
    Vec3 v{1.0, 2.0, 3.0};

    std::ostringstream oss;
    oss << v;

    EXPECT_EQ(oss.str(), "(1, 2, 3)");
}

TEST(VectorTest, Print) {
    Vec3 v{1.0, 2.0, 3.0};

    std::ostringstream capture;
    auto* old_buf = std::cout.rdbuf(capture.rdbuf());

    v.print();

    std::cout.rdbuf(old_buf);

    EXPECT_EQ(capture.str(), "[Vector] - (1, 2, 3)\n");
}

// -----------------------------------------------------------------------------
// Copy and move semantics
// -----------------------------------------------------------------------------

TEST(VectorTest, CopySemantics) {
    Vec3 v1{1.0, 2.0, 3.0};

    Vec3 v2 = v1;
    EXPECT_EQ(v1, v2);

    Vec3 v3;
    v3 = v1;
    EXPECT_EQ(v1, v3);

    v2[0] = 999.0;

    EXPECT_NE(v1, v2);
    EXPECT_EQ(v1[0], 1.0);
}

TEST(VectorTest, MoveSemantics) {
    Vec3 v1{1.0, 2.0, 3.0};
    Vec3 original = v1;

    Vec3 v2 = std::move(v1);
    EXPECT_EQ(v2, original);

    Vec3 v3;
    Vec3 v4{4.0, 5.0, 6.0};
    Vec3 v4_copy = v4;

    v3 = std::move(v4);

    EXPECT_EQ(v3, v4_copy);
}

// -----------------------------------------------------------------------------
// Randomized property testing
// -----------------------------------------------------------------------------

TEST(VectorTest, RandomNormalizationProperty) {
    std::mt19937 rng(12345);
    std::uniform_real_distribution<double> dist(-1000.0, 1000.0);

    for (int i = 0; i < 10000; ++i) {
        Vec3 v{dist(rng), dist(rng), dist(rng)};

        if (v.magnitude() < 1e-12) {
            continue;
        }

        Vec3 n = v.normalize();

        EXPECT_NEAR(n.magnitude(), 1.0, 1e-10);
    }
}

TEST(VectorTest, RandomDotProductSymmetry) {
    std::mt19937 rng(67890);
    std::uniform_real_distribution<double> dist(-1000.0, 1000.0);

    for (int i = 0; i < 10000; ++i) {
        Vec3 a{dist(rng), dist(rng), dist(rng)};
        Vec3 b{dist(rng), dist(rng), dist(rng)};

        EXPECT_NEAR(dotProduct(a, b), dotProduct(b, a), kTol);
    }
}

TEST(VectorTest, RandomCrossProductOrthogonality) {
    std::mt19937 rng(54321);
    std::uniform_real_distribution<double> dist(-100.0, 100.0);

    for (int i = 0; i < 1000; ++i) {  // 10x reduction
        Vec3 a{dist(rng), dist(rng), dist(rng)};
        Vec3 b{dist(rng), dist(rng), dist(rng)};

        Vec3 c = crossProduct(a, b);

        EXPECT_NEAR(dotProduct(a, c), 0.0, 1e-8);
        EXPECT_NEAR(dotProduct(b, c), 0.0, 1e-8);
    }
}

// -----------------------------------------------------------------------------
// Integer vector behavior
// -----------------------------------------------------------------------------

TEST(VectorTest, IntegerVectorNormalization) {
    Vector<int, 3> v{3, 4, 0};

    // magnitude() = 5, integer division truncates.
    auto n = v.normalize();

    auto test = Vector<int, 3>({0, 0, 0});

    EXPECT_EQ(n, test);
}