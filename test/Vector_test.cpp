// GOOGLETEST
#include <gtest/gtest.h>

// STL
#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <random>
#include <sstream>
#include <utility>

// LOCAL
#include <RTB/Point.hpp>
#include <RTB/Vector.hpp>

using Vec3 = RTB::Vector<double, 3>;
using Point3 = RTB::Point<double, 3>;

namespace {

constexpr double epsilon = 1e-12;

void expectVecNear(const Vec3& actual,
                   const Vec3& expected,
                   double tol = epsilon) {
    for (std::size_t i = 0; i < 3; ++i) {
        EXPECT_NEAR(actual[i], expected[i], tol) << "Mismatch at index " << i;
    }
}

}  // namespace

// -----------------------------------------------------------------------------
// Constructors
// -----------------------------------------------------------------------------

TEST(VectorTest, DefaultConstructor) {
    Vec3 vector;
    for (std::size_t i = 0; i < 3; ++i) {
        EXPECT_EQ(vector[i], 0.0);
    }
}

TEST(VectorTest, InitializerListConstructor) {
    Vec3 vector{1.0, 2.0, 3.0};

    EXPECT_EQ(vector[0], 1.0);
    EXPECT_EQ(vector[1], 2.0);
    EXPECT_EQ(vector[2], 3.0);
}

TEST(VectorTest, ArrayConstructor) {
    const std::array<double, 3> arr{4.0, 5.0, 6.0};
    const Vec3 vector(arr);

    EXPECT_EQ(vector.getComponents(), arr);
}

TEST(VectorTest, PointConstructor) {
    const Point3 point{1.0, 2.0, 3.0};
    const Vec3 vector(point);

    EXPECT_EQ(vector, Vec3({1.0, 2.0, 3.0}));
}

TEST(VectorTest, TwoPointConstructor) {
    const Point3 p1{1.0, 1.0, 1.0};
    const Point3 p2{4.0, 5.0, 6.0};

    const Vec3 vector(p1, p2);

    EXPECT_EQ(vector, Vec3({3.0, 4.0, 5.0}));
}

// -----------------------------------------------------------------------------
// Basic utilities
// -----------------------------------------------------------------------------

TEST(VectorTest, Size) {
    EXPECT_EQ(Vec3::size(), 3U);
}

TEST(VectorTest, SetAndGetComponents) {
    Vec3 vector;
    const std::array<double, 3> components{7.0, 8.0, 9.0};

    vector.setComponents(components);

    EXPECT_EQ(vector.getComponents(), components);
}

TEST(VectorTest, IndexOperator) {
    Vec3 vector{1.0, 2.0, 3.0};

    EXPECT_EQ(vector[0], 1.0);
    EXPECT_EQ(vector[1], 2.0);
    EXPECT_EQ(vector[2], 3.0);

    vector[1] = 10.0;
    EXPECT_EQ(vector[1], 10.0);
}

// -----------------------------------------------------------------------------
// Scalar arithmetic
// -----------------------------------------------------------------------------

TEST(VectorTest, ArithmeticOperatorsScalar) {
    const Vec3 vector{1.0, 2.0, 3.0};

    EXPECT_EQ(vector + 2.0, Vec3({3.0, 4.0, 5.0}));
    EXPECT_EQ(vector - 1.0, Vec3({0.0, 1.0, 2.0}));
    EXPECT_EQ(vector * 2.0, Vec3({2.0, 4.0, 6.0}));
    EXPECT_EQ(vector / 2.0, Vec3({0.5, 1.0, 1.5}));
}

TEST(VectorTest, ScalarOperatorsCommutative) {
    const Vec3 vector{1.0, 2.0, 3.0};

    EXPECT_EQ(2.0 + vector, Vec3({3.0, 4.0, 5.0}));
    EXPECT_EQ(vector + 2.0, 2.0 + vector);

    EXPECT_EQ(3.0 * vector, Vec3({3.0, 6.0, 9.0}));
    EXPECT_EQ(vector * 3.0, 3.0 * vector);

    EXPECT_EQ(5.0 - vector, Vec3({4.0, 3.0, 2.0}));
}

// -----------------------------------------------------------------------------
// RTB::Vector arithmetic
// -----------------------------------------------------------------------------

TEST(VectorTest, ArithmeticOperatorsVector) {
    const Vec3 v1{1.0, 2.0, 3.0};
    const Vec3 v2{4.0, 5.0, 6.0};

    EXPECT_EQ(v1 + v2, Vec3({5.0, 7.0, 9.0}));
    EXPECT_EQ(v2 - v1, Vec3({3.0, 3.0, 3.0}));
    EXPECT_EQ(v1 * v2, Vec3({4.0, 10.0, 18.0}));
}

TEST(VectorTest, ElementWiseDivision) {
    Vec3 vector_a{6.0, 8.0, 10.0};
    const Vec3 vector_b{2.0, 4.0, 5.0};

    EXPECT_EQ(vector_a / vector_b, Vec3({3.0, 2.0, 2.0}));

    vector_a /= vector_b;
    EXPECT_EQ(vector_a, Vec3({3.0, 2.0, 2.0}));
}

// -----------------------------------------------------------------------------
// Compound assignment
// -----------------------------------------------------------------------------

TEST(VectorTest, CompoundAssignmentOperators) {
    Vec3 vector{1.0, 2.0, 3.0};

    vector += 1.0;
    EXPECT_EQ(vector, Vec3({2.0, 3.0, 4.0}));

    vector -= 1.0;
    EXPECT_EQ(vector, Vec3({1.0, 2.0, 3.0}));

    vector *= 2.0;
    EXPECT_EQ(vector, Vec3({2.0, 4.0, 6.0}));

    vector /= 2.0;
    EXPECT_EQ(vector, Vec3({1.0, 2.0, 3.0}));

    const Vec3 other{1.0, 1.0, 1.0};

    vector += other;
    EXPECT_EQ(vector, Vec3({2.0, 3.0, 4.0}));

    vector -= other;
    EXPECT_EQ(vector, Vec3({1.0, 2.0, 3.0}));

    vector *= other;
    EXPECT_EQ(vector, Vec3({1.0, 2.0, 3.0}));

    vector /= other;
    EXPECT_EQ(vector, Vec3({1.0, 2.0, 3.0}));
}

// -----------------------------------------------------------------------------
// Unary operations
// -----------------------------------------------------------------------------

TEST(VectorTest, UnaryMinus) {
    const Vec3 vector{1.0, -2.0, 3.0};

    const Vec3 negated = -vector;

    EXPECT_EQ(negated, Vec3({-1.0, 2.0, -3.0}));
    EXPECT_EQ(vector, Vec3({1.0, -2.0, 3.0}));
}

TEST(VectorTest, Invert) {
    Vec3 vector{1.0, -2.0, 3.0};

    const Vec3& result = vector.invert();

    EXPECT_EQ(vector, Vec3({-1.0, 2.0, -3.0}));
    EXPECT_EQ(&result, &vector);
}

// -----------------------------------------------------------------------------
// Magnitude and normalization
// -----------------------------------------------------------------------------

TEST(VectorTest, LengthCalculations) {
    const Vec3 vector{3.0, 4.0, 0.0};

    EXPECT_DOUBLE_EQ(vector.magnitudeSquared(), 25.0);
    EXPECT_DOUBLE_EQ(vector.magnitude(), 5.0);
}

TEST(VectorTest, Normalize) {
    const Vec3 vector{0.0, 3.0, 4.0};
    const Vec3 original = vector;

    const Vec3 normalized = vector.normalize();

    expectVecNear(normalized, Vec3({0.0, 0.6, 0.8}));
    EXPECT_NEAR(normalized.magnitude(), 1.0, epsilon);

    EXPECT_EQ(vector, original);
}

TEST(VectorTest, NormalizeInPlace) {
    Vec3 vector{0.0, 3.0, 4.0};

    const Vec3& result = vector.normalizeInPlace();

    expectVecNear(vector, Vec3({0.0, 0.6, 0.8}));
    EXPECT_NEAR(vector.magnitude(), 1.0, epsilon);
    EXPECT_EQ(&result, &vector);
}

TEST(VectorTest, NormalizeZeroVector) {
    Vec3 zero{0.0, 0.0, 0.0};

    EXPECT_EQ(zero.normalize(), zero);

    zero.normalizeInPlace();
    EXPECT_EQ(zero, Vec3({0.0, 0.0, 0.0}));
}

TEST(VectorTest, ChainOperations) {
    Vec3 vector{0.0, 6.0, 8.0};

    vector.normalizeInPlace().invert();

    expectVecNear(vector, Vec3({0.0, -0.6, -0.8}));
}

// -----------------------------------------------------------------------------
// Dot product
// -----------------------------------------------------------------------------

TEST(VectorTest, DotProduct) {
    const Vec3 v1{1.0, 2.0, 3.0};
    const Vec3 v2{4.0, -5.0, 6.0};

    EXPECT_DOUBLE_EQ(dotProduct(v1, v2), 12.0);
}

TEST(VectorTest, DotProductMixedTypes) {
    const RTB::Vector<float, 3> vf{1.0F, 2.0F, 3.0F};
    const RTB::Vector<double, 3> vd{4.0, -5.0, 6.0};

    auto result = dotProduct(vf, vd);

    EXPECT_DOUBLE_EQ(result, 12.0);
}

TEST(VectorTest, DotProductDistributive) {
    const Vec3 vector_a{1.0, 2.0, 3.0};
    const Vec3 vector_b{4.0, 5.0, 6.0};
    const Vec3 vector_c{7.0, 8.0, 9.0};

    EXPECT_NEAR(dotProduct(vector_a, vector_b + vector_c),
                dotProduct(vector_a, vector_b) + dotProduct(vector_a, vector_c),
                epsilon);
}

// -----------------------------------------------------------------------------
// Cross product
// -----------------------------------------------------------------------------

TEST(VectorTest, CrossProduct) {
    const Vec3 v1{1.0, 0.0, 0.0};
    const Vec3 v2{0.0, 1.0, 0.0};

    EXPECT_EQ(crossProduct(v1, v2), Vec3({0.0, 0.0, 1.0}));
    EXPECT_EQ(crossProduct(v2, v1), Vec3({0.0, 0.0, -1.0}));
}

TEST(VectorTest, CrossProductMixedTypes) {
    const RTB::Vector<float, 3> vf{1.0F, 0.0F, 0.0F};
    const RTB::Vector<double, 3> vd{0.0, 1.0, 0.0};

    EXPECT_EQ(crossProduct(vf, vd), Vec3({0.0, 0.0, 1.0}));
}

TEST(VectorTest, CrossProductOrthogonality) {
    const Vec3 vector_a{1.0, 2.0, 3.0};
    const Vec3 vector_b{4.0, 5.0, 6.0};

    const Vec3 vector_c = crossProduct(vector_a, vector_b);

    EXPECT_NEAR(dotProduct(vector_a, vector_c), 0.0, epsilon);
    EXPECT_NEAR(dotProduct(vector_b, vector_c), 0.0, epsilon);
}

TEST(VectorTest, CrossProductLagrangeIdentity) {
    const Vec3 vector_a{1.0, 2.0, 3.0};
    const Vec3 vector_b{4.0, 5.0, 6.0};

    const Vec3 vector_c = crossProduct(vector_a, vector_b);

    const double lhs = vector_c.magnitudeSquared();
    const double rhs =
        (vector_a.magnitudeSquared() * vector_b.magnitudeSquared()) -
        std::pow(dotProduct(vector_a, vector_b), 2.0);

    EXPECT_NEAR(lhs, rhs, epsilon);
}

// -----------------------------------------------------------------------------
// Equality
// -----------------------------------------------------------------------------

TEST(VectorTest, EqualityOperators) {
    const Vec3 v1{1.0, 2.0, 3.0};
    const Vec3 v2{1.0, 2.0, 3.0};
    const Vec3 v3{3.0, 2.0, 1.0};

    EXPECT_TRUE(v1 == v2);
    EXPECT_FALSE(v1 != v2);

    EXPECT_TRUE(v1 != v3);
    EXPECT_FALSE(v1 == v3);
}

// -----------------------------------------------------------------------------
// Stream output and print
// -----------------------------------------------------------------------------

TEST(VectorTest, StreamOutput) {
    const Vec3 vector{1.0, 2.0, 3.0};

    std::ostringstream oss;
    oss << vector;

    EXPECT_EQ(oss.str(), "(1, 2, 3)");
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
    const Vec3 original = v1;

    const Vec3 v2 = std::move(v1);  // NOLINT(performance-move-const-arg)
    EXPECT_EQ(v2, original);

    Vec3 v3;
    Vec3 v4{4.0, 5.0, 6.0};
    const Vec3 v4_copy = v4;

    v3 = std::move(v4);  // NOLINT(performance-move-const-arg)

    EXPECT_EQ(v3, v4_copy);
}

// -----------------------------------------------------------------------------
// Randomized property testing
// -----------------------------------------------------------------------------

TEST(VectorTest, RandomNormalizationProperty) {
    std::mt19937 rng(12345);  // NOLINT
    std::uniform_real_distribution<double> dist(-1000.0, 1000.0);

    for (int i = 0; i < 10000; ++i) {
        const Vec3 vector{dist(rng), dist(rng), dist(rng)};

        if (vector.magnitude() < 1e-12) {
            continue;
        }

        const Vec3 normal = vector.normalize();

        EXPECT_NEAR(normal.magnitude(), 1.0, 1e-10);
    }
}

TEST(VectorTest, RandomDotProductSymmetry) {
    std::mt19937 rng(67890);  // NOLINT
    std::uniform_real_distribution<double> dist(-1000.0, 1000.0);

    for (int i = 0; i < 10000; ++i) {
        const Vec3 vector_a{dist(rng), dist(rng), dist(rng)};
        const Vec3 vector_b{dist(rng), dist(rng), dist(rng)};

        EXPECT_NEAR(dotProduct(vector_a, vector_b),
                    dotProduct(vector_b, vector_a),
                    epsilon);
    }
}

TEST(VectorTest, RandomCrossProductOrthogonality) {
    std::mt19937 rng(54321);  // NOLINT
    std::uniform_real_distribution<double> dist(-100.0, 100.0);

    for (int i = 0; i < 1000; ++i) {  // 10x reduction
        const Vec3 vector_a{dist(rng), dist(rng), dist(rng)};
        const Vec3 vector_b{dist(rng), dist(rng), dist(rng)};

        const Vec3 vector_c = crossProduct(vector_a, vector_b);

        EXPECT_NEAR(dotProduct(vector_a, vector_c), 0.0, 1e-8);
        EXPECT_NEAR(dotProduct(vector_b, vector_c), 0.0, 1e-8);
    }
}

// -----------------------------------------------------------------------------
// Integer vector behavior
// -----------------------------------------------------------------------------

TEST(VectorTest, IntegerVectorNormalization) {
    const RTB::Vector<int, 3> vector{3, 4, 0};

    // magnitude() = 5, integer division truncates.
    auto normal = vector.normalize();

    auto test = RTB::Vector<int, 3>({0, 0, 0});

    EXPECT_EQ(normal, test);
}