// GOOGLETEST
#include <gtest/gtest.h>

// STL
#include <cmath>
#include <cstddef>
#include <optional>

// LOCAL
#include <RTB/Plane.hpp>
#include <RTB/Point.hpp>
#include <RTB/Ray.hpp>
#include <RTB/Vector.hpp>

namespace {

using T = double;
using Plane3 = RTB::Plane<T>;
using Point3 = RTB::Point<T, 3>;
using Vector3 = RTB::Vector<T, 3>;
using Ray3 = RTB::Ray<T, 3>;

constexpr T k_eps = 1e-9;

// ------------------------------------------------------------
// Helper Functions
// ------------------------------------------------------------

void expectNear(T actual, T expected, T eps = k_eps) {
    EXPECT_NEAR(actual, expected, eps);
}

void expectPointNear(const Point3& actual,
                     const Point3& expected,
                     T eps = k_eps) {
    for (size_t i = 0; i < 3; ++i) {
        EXPECT_NEAR(actual[i], expected[i], eps) << "Mismatch at index " << i;
    }
}

void expectVectorNear(const Vector3& actual,
                      const Vector3& expected,
                      T eps = k_eps) {
    for (size_t i = 0; i < 3; ++i) {
        EXPECT_NEAR(actual[i], expected[i], eps) << "Mismatch at index " << i;
    }
}

void expectPlaneCoefficientsNear(const Plane3& plane,
                                 T coeff_a,
                                 T coeff_b,
                                 T coeff_c,
                                 T coeff_d,
                                 T eps = k_eps) {
    auto coeffs = plane.getCoefficients();
    EXPECT_NEAR(coeffs[0], coeff_a, eps);
    EXPECT_NEAR(coeffs[1], coeff_b, eps);
    EXPECT_NEAR(coeffs[2], coeff_c, eps);
    EXPECT_NEAR(coeffs[3], coeff_d, eps);
}

T evaluatePlane(const Plane3& plane, const Point3& point) {
    auto coeffs = plane.getCoefficients();
    return (coeffs[0] * point[0]) + (coeffs[1] * point[1]) +
           (coeffs[2] * point[2]) + coeffs[3];
}

bool isPointOnPlane(const Plane3& plane, const Point3& point, T eps = k_eps) {
    return std::abs(evaluatePlane(plane, point)) < eps;
}

bool areVectorsParallel(const Vector3& vector_a,
                        const Vector3& vector_b,
                        T eps = k_eps) {
    const Vector3 cross = RTB::crossProduct(vector_a, vector_b);
    return cross.magnitude() < eps;
}

bool areVectorsEqualOrOpposite(const Vector3& vector_a,
                               const Vector3& vector_b,
                               T eps = k_eps) {
    bool equal = true;
    bool opposite = true;

    for (size_t i = 0; i < 3; ++i) {
        if (std::abs(vector_a[i] - vector_b[i]) > eps) {
            equal = false;
        }
        if (std::abs(vector_a[i] + vector_b[i]) > eps) {
            opposite = false;
        }
    }

    return equal || opposite;
}

// ------------------------------------------------------------
// Constructor Tests
// ------------------------------------------------------------

TEST(PlaneTest, DefaultConstructorInitializesToZero) {
    const Plane3 plane;
    expectPlaneCoefficientsNear(plane, 0.0, 0.0, 0.0, 0.0);
}

TEST(PlaneTest, CoefficientConstructorStoresValues) {
    const Plane3 plane(1.0, 2.0, 3.0, 4.0);
    expectPlaneCoefficientsNear(plane, 1.0, 2.0, 3.0, 4.0);
}

TEST(PlaneTest, ThreePointConstructorXYPlane) {
    const Point3 point_a{0.0, 0.0, 0.0};
    const Point3 point_b{1.0, 0.0, 0.0};
    const Point3 point_c{0.0, 1.0, 0.0};

    const Plane3 plane(point_a, point_b, point_c);

    EXPECT_TRUE(isPointOnPlane(plane, point_a));
    EXPECT_TRUE(isPointOnPlane(plane, point_b));
    EXPECT_TRUE(isPointOnPlane(plane, point_c));

    auto normal = plane.getNormalVector();
    EXPECT_TRUE(areVectorsParallel(normal, Vector3{0.0, 0.0, 1.0}));
}

TEST(PlaneTest, ThreePointConstructorGeneralPlane) {
    const Point3 point_a{1.0, 0.0, 0.0};
    const Point3 point_b{0.0, 1.0, 0.0};
    const Point3 point_c{0.0, 0.0, 1.0};

    const Plane3 plane(point_a, point_b, point_c);

    EXPECT_TRUE(isPointOnPlane(plane, point_a));
    EXPECT_TRUE(isPointOnPlane(plane, point_b));
    EXPECT_TRUE(isPointOnPlane(plane, point_c));
}

// ------------------------------------------------------------
// invert()
// ------------------------------------------------------------

TEST(PlaneTest, InvertNegatesAllCoefficients) {
    Plane3 plane(1.0, -2.0, 3.0, -4.0);
    plane.invert();

    expectPlaneCoefficientsNear(plane, -1.0, 2.0, -3.0, 4.0);
}

// ------------------------------------------------------------
// normalize()
// ------------------------------------------------------------

TEST(PlaneTest, NormalizeProducesUnitNormal) {
    Plane3 plane(2.0, 0.0, 0.0, -4.0);
    plane.normalize();

    expectPlaneCoefficientsNear(plane, 1.0, 0.0, 0.0, -2.0);

    auto normal = plane.getNormalVector();
    EXPECT_NEAR(normal.magnitude(), 1.0, k_eps);
}

TEST(PlaneTest, NormalizeZeroPlaneDoesNothing) {
    Plane3 plane(0.0, 0.0, 0.0, 5.0);
    plane.normalize();

    expectPlaneCoefficientsNear(plane, 0.0, 0.0, 0.0, 5.0);
}

// ------------------------------------------------------------
// getNormalVector()
// ------------------------------------------------------------

TEST(PlaneTest, GetNormalVectorReturnsFirstThreeCoefficients) {
    const Plane3 plane(4.0, 5.0, 6.0, 7.0);

    auto normal = plane.getNormalVector();

    expectVectorNear(normal, Vector3{4.0, 5.0, 6.0});
}

// ------------------------------------------------------------
// getIntersection()
// ------------------------------------------------------------

TEST(PlaneTest, GetIntersectionReturnsCorrectDistance) {
    // Plane z = 5
    Plane3 plane(0.0, 0.0, 1.0, -5.0);
    plane.normalize();

    const Ray3 ray(Point3{0.0, 0.0, 0.0}, Vector3{0.0, 0.0, 1.0});

    auto step = plane.getIntersection(ray);

    if (step.has_value()) {
        EXPECT_NEAR(*step, 5.0, k_eps);
    } else {
        FAIL() << "Expected intersection but got none";
    }
}

TEST(PlaneTest, GetIntersectionRayStartsOnPlaneAndLiesInPlaneReturnsNullopt) {
    Plane3 plane(0.0, 0.0, 1.0, -5.0);
    plane.normalize();

    // Origin lies on the plane z = 5, and direction is parallel to the plane.
    // In Plane::getIntersection():
    //   denominator = n · d = 0
    // so the function treats the ray as parallel and returns std::nullopt,
    // even though the ray starts on the plane.
    const Ray3 ray(Point3{1.0, 2.0, 5.0}, Vector3{1.0, 0.0, 0.0});

    auto step = plane.getIntersection(ray);

    EXPECT_FALSE(step.has_value());
}

TEST(PlaneTest, GetIntersectionParallelRayReturnsNullopt) {
    Plane3 plane(0.0, 0.0, 1.0, -5.0);
    plane.normalize();

    const Ray3 ray(Point3{0.0, 0.0, 0.0}, Vector3{1.0, 0.0, 0.0});

    auto step = plane.getIntersection(ray);

    EXPECT_FALSE(step.has_value());
}

TEST(PlaneTest, GetIntersectionBehindRayReturnsNullopt) {
    Plane3 plane(0.0, 0.0, 1.0, -5.0);
    plane.normalize();

    const Ray3 ray(Point3{0.0, 0.0, 10.0}, Vector3{0.0, 0.0, 1.0});

    auto step = plane.getIntersection(ray);

    EXPECT_FALSE(step.has_value());
}

// ------------------------------------------------------------
// reflect()
// ------------------------------------------------------------

TEST(PlaneTest, ReflectOffHorizontalPlane) {
    // Plane z = 0
    Plane3 plane(0.0, 0.0, 1.0, 0.0);
    plane.normalize();

    Ray3 ray(Point3{0.0, 0.0, 1.0}, Vector3{0.0, 0.0, -1.0});

    auto step = plane.getIntersection(ray);
    if (step.has_value()) {
        plane.reflect(ray, *step);
    } else {
        FAIL() << "Expected value but got none";
    }

    // Origin should be at intersection point
    expectPointNear(ray.getOrigin(), Point3{0.0, 0.0, 0.0});

    // Direction should be reflected upward
    expectVectorNear(ray.getDirection(), Vector3{0.0, 0.0, 1.0});
}

TEST(PlaneTest, ReflectAt45Degrees) {
    // Plane y = 0
    Plane3 plane(0.0, 1.0, 0.0, 0.0);
    plane.normalize();

    Vector3 incoming{1.0, -1.0, 0.0};
    incoming.normalizeInPlace();

    Ray3 ray(Point3{0.0, 1.0, 0.0}, incoming);

    auto step = plane.getIntersection(ray);
    if (step.has_value()) {
        plane.reflect(ray, *step);
    } else {
        FAIL() << "Expected value but got none";
    }

    Vector3 expected{1.0, 1.0, 0.0};
    expected.normalizeInPlace();

    expectVectorNear(ray.getDirection(), expected);
}

TEST(PlaneTest, ReflectedDirectionIsNormalized) {
    Plane3 plane(0.0, 0.0, 1.0, 0.0);
    plane.normalize();

    const Vector3 dir{2.0, 0.0, -2.0};
    Ray3 ray(Point3{0.0, 0.0, 1.0}, dir);

    auto step = plane.getIntersection(ray);
    ASSERT_TRUE(step.has_value());
    if (step.has_value()) {
        plane.reflect(ray, step.value());
    }

    EXPECT_NEAR(ray.getDirection().magnitude(), 1.0, k_eps);
}

// ------------------------------------------------------------
// intersectPlanes()
// ------------------------------------------------------------

TEST(PlaneTest, IntersectPlanesReturnsLineForOrthogonalPlanes) {
    // x = 0
    Plane3 p1(1.0, 0.0, 0.0, 0.0);
    // y = 0
    Plane3 p2(0.0, 1.0, 0.0, 0.0);

    p1.normalize();
    p2.normalize();

    auto line = RTB::intersectPlanes(p1, p2);

    if (line.has_value()) {
        // Origin should satisfy both planes
        EXPECT_TRUE(isPointOnPlane(p1, line->getOrigin()));
        EXPECT_TRUE(isPointOnPlane(p2, line->getOrigin()));
    } else {
        FAIL() << "Expected value but got none";
    }

    // Direction should be parallel to z-axis
    EXPECT_TRUE(
        areVectorsParallel(line->getDirection(), Vector3{0.0, 0.0, 1.0}));
}

TEST(PlaneTest, IntersectPlanesReturnsCorrectLine) {
    // x = 1
    Plane3 p1(1.0, 0.0, 0.0, -1.0);
    // y = 2
    Plane3 p2(0.0, 1.0, 0.0, -2.0);

    p1.normalize();
    p2.normalize();

    auto line = RTB::intersectPlanes(p1, p2);

    Point3 origin;
    if (line.has_value()) {
        origin = line->getOrigin();
    } else {
        FAIL() << "Expected value but got none";
    }

    expectNear(origin[0], 1.0);
    expectNear(origin[1], 2.0);

    EXPECT_TRUE(
        areVectorsParallel(line->getDirection(), Vector3{0.0, 0.0, 1.0}));
}

TEST(PlaneTest, IntersectPlanesParallelReturnsNullopt) {
    // x = 1
    Plane3 p1(1.0, 0.0, 0.0, -1.0);
    // x = 2
    Plane3 p2(1.0, 0.0, 0.0, -2.0);

    p1.normalize();
    p2.normalize();

    auto line = RTB::intersectPlanes(p1, p2);

    EXPECT_FALSE(line.has_value());
}

TEST(PlaneTest, IntersectPlanesCoincidentReturnsNullopt) {
    Plane3 p1(1.0, 2.0, 3.0, 4.0);
    Plane3 p2(2.0, 4.0, 6.0, 8.0);

    p1.normalize();
    p2.normalize();

    auto line = RTB::intersectPlanes(p1, p2);

    EXPECT_FALSE(line.has_value());
}

TEST(PlaneTest, IntersectPlanesDirectionMatchesCrossProduct) {
    Plane3 p1(1.0, 2.0, 3.0, -4.0);
    Plane3 p2(2.0, -1.0, 1.0, 5.0);

    p1.normalize();
    p2.normalize();

    auto line = RTB::intersectPlanes(p1, p2);

    Vector3 expected =
        RTB::crossProduct(p1.getNormalVector(), p2.getNormalVector());
    expected.normalizeInPlace();

    if (line.has_value()) {
        EXPECT_TRUE(areVectorsEqualOrOpposite(line->getDirection(), expected));
    } else {
        FAIL() << "Expected value but got none";
    }
}

// ------------------------------------------------------------
// Regression / Numerical Stability Tests
// ------------------------------------------------------------

TEST(PlaneTest, ThreePointConstructorThenNormalizeProducesUnitNormal) {
    const Point3 point_a{1.0, 1.0, 1.0};
    const Point3 point_b{2.0, 1.0, 1.0};
    const Point3 point_c{1.0, 3.0, 1.0};

    Plane3 plane(point_a, point_b, point_c);
    plane.normalize();

    EXPECT_NEAR(plane.getNormalVector().magnitude(), 1.0, k_eps);

    EXPECT_TRUE(isPointOnPlane(plane, point_a));
    EXPECT_TRUE(isPointOnPlane(plane, point_b));
    EXPECT_TRUE(isPointOnPlane(plane, point_c));
}

TEST(PlaneTest, ReflectionPreservesIncidentAngleMagnitude) {
    Plane3 plane(0.0, 0.0, 1.0, 0.0);
    plane.normalize();

    Vector3 dir{1.0, 0.0, -1.0};
    dir.normalizeInPlace();

    Ray3 ray(Point3{0.0, 0.0, 1.0}, dir);

    auto step = plane.getIntersection(ray);
    ASSERT_TRUE(step.has_value());

    if (step.has_value()) {
        plane.reflect(ray, *step);
    } else {
        FAIL() << "Expected value but got none";
    }

    EXPECT_NEAR(ray.getDirection().magnitude(), 1.0, k_eps);

    Vector3 expected{1.0, 0.0, 1.0};
    expected.normalizeInPlace();

    expectVectorNear(ray.getDirection(), expected);
}

}  // namespace