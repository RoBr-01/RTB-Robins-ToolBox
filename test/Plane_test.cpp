#include <gtest/gtest.h>

#include <RTB/Plane.hpp>
#include <RTB/Point.hpp>
#include <RTB/Ray.hpp>
#include <RTB/Vector.hpp>
#include <cmath>
#include <cstddef>
#include <optional>

namespace {

using T = double;
using Plane3 = RTB::Plane<T>;
using Point3 = RTB::Point<T, 3>;
using Vector3 = RTB::Vector<T, 3>;
using Ray3 = RTB::Ray<T, 3>;

constexpr T kEps = 1e-9;

// ------------------------------------------------------------
// Helper Functions
// ------------------------------------------------------------

void ExpectNear(T actual, T expected, T eps = kEps) {
    EXPECT_NEAR(actual, expected, eps);
}

void ExpectPointNear(const Point3& actual,
                     const Point3& expected,
                     T eps = kEps) {
    for (size_t i = 0; i < 3; ++i) {
        EXPECT_NEAR(actual[i], expected[i], eps) << "Mismatch at index " << i;
    }
}

void ExpectVectorNear(const Vector3& actual,
                      const Vector3& expected,
                      T eps = kEps) {
    for (size_t i = 0; i < 3; ++i) {
        EXPECT_NEAR(actual[i], expected[i], eps) << "Mismatch at index " << i;
    }
}

void ExpectPlaneCoefficientsNear(
    const Plane3& plane, T a, T b, T c, T d, T eps = kEps) {
    auto coeffs = plane.getCoefficients();
    EXPECT_NEAR(coeffs[0], a, eps);
    EXPECT_NEAR(coeffs[1], b, eps);
    EXPECT_NEAR(coeffs[2], c, eps);
    EXPECT_NEAR(coeffs[3], d, eps);
}

T EvaluatePlane(const Plane3& plane, const Point3& point) {
    auto coeffs = plane.getCoefficients();
    return (coeffs[0] * point[0]) + (coeffs[1] * point[1]) +
           (coeffs[2] * point[2]) + coeffs[3];
}

bool IsPointOnPlane(const Plane3& plane, const Point3& point, T eps = kEps) {
    return std::abs(EvaluatePlane(plane, point)) < eps;
}

bool AreVectorsParallel(const Vector3& a, const Vector3& b, T eps = kEps) {
    const Vector3 cross = RTB::crossProduct(a, b);
    return cross.magnitude() < eps;
}

bool AreVectorsEqualOrOpposite(const Vector3& a,
                               const Vector3& b,
                               T eps = kEps) {
    bool equal = true;
    bool opposite = true;

    for (size_t i = 0; i < 3; ++i) {
        if (std::abs(a[i] - b[i]) > eps) {
            equal = false;
        }
        if (std::abs(a[i] + b[i]) > eps) {
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
    ExpectPlaneCoefficientsNear(plane, 0.0, 0.0, 0.0, 0.0);
}

TEST(PlaneTest, CoefficientConstructorStoresValues) {
    const Plane3 plane(1.0, 2.0, 3.0, 4.0);
    ExpectPlaneCoefficientsNear(plane, 1.0, 2.0, 3.0, 4.0);
}

TEST(PlaneTest, ThreePointConstructorXYPlane) {
    const Point3 a{0.0, 0.0, 0.0};
    const Point3 b{1.0, 0.0, 0.0};
    const Point3 c{0.0, 1.0, 0.0};

    const Plane3 plane(a, b, c);

    EXPECT_TRUE(IsPointOnPlane(plane, a));
    EXPECT_TRUE(IsPointOnPlane(plane, b));
    EXPECT_TRUE(IsPointOnPlane(plane, c));

    auto n = plane.getNormalVector();
    EXPECT_TRUE(AreVectorsParallel(n, Vector3{0.0, 0.0, 1.0}));
}

TEST(PlaneTest, ThreePointConstructorGeneralPlane) {
    const Point3 a{1.0, 0.0, 0.0};
    const Point3 b{0.0, 1.0, 0.0};
    const Point3 c{0.0, 0.0, 1.0};

    const Plane3 plane(a, b, c);

    EXPECT_TRUE(IsPointOnPlane(plane, a));
    EXPECT_TRUE(IsPointOnPlane(plane, b));
    EXPECT_TRUE(IsPointOnPlane(plane, c));
}

// ------------------------------------------------------------
// invert()
// ------------------------------------------------------------

TEST(PlaneTest, InvertNegatesAllCoefficients) {
    Plane3 plane(1.0, -2.0, 3.0, -4.0);
    plane.invert();

    ExpectPlaneCoefficientsNear(plane, -1.0, 2.0, -3.0, 4.0);
}

// ------------------------------------------------------------
// normalize()
// ------------------------------------------------------------

TEST(PlaneTest, NormalizeProducesUnitNormal) {
    Plane3 plane(2.0, 0.0, 0.0, -4.0);
    plane.normalize();

    ExpectPlaneCoefficientsNear(plane, 1.0, 0.0, 0.0, -2.0);

    auto n = plane.getNormalVector();
    EXPECT_NEAR(n.magnitude(), 1.0, kEps);
}

TEST(PlaneTest, NormalizeZeroPlaneDoesNothing) {
    Plane3 plane(0.0, 0.0, 0.0, 5.0);
    plane.normalize();

    ExpectPlaneCoefficientsNear(plane, 0.0, 0.0, 0.0, 5.0);
}

// ------------------------------------------------------------
// getNormalVector()
// ------------------------------------------------------------

TEST(PlaneTest, GetNormalVectorReturnsFirstThreeCoefficients) {
    const Plane3 plane(4.0, 5.0, 6.0, 7.0);

    auto n = plane.getNormalVector();

    ExpectVectorNear(n, Vector3{4.0, 5.0, 6.0});
}

// ------------------------------------------------------------
// getIntersection()
// ------------------------------------------------------------

TEST(PlaneTest, GetIntersectionReturnsCorrectDistance) {
    // Plane z = 5
    Plane3 plane(0.0, 0.0, 1.0, -5.0);
    plane.normalize();

    const Ray3 ray(Point3{0.0, 0.0, 0.0}, Vector3{0.0, 0.0, 1.0});

    auto t = plane.getIntersection(ray);

    ASSERT_TRUE(t.has_value());
    EXPECT_NEAR(*t, 5.0, kEps);
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

    auto t = plane.getIntersection(ray);

    EXPECT_FALSE(t.has_value());
}

TEST(PlaneTest, GetIntersectionParallelRayReturnsNullopt) {
    Plane3 plane(0.0, 0.0, 1.0, -5.0);
    plane.normalize();

    const Ray3 ray(Point3{0.0, 0.0, 0.0}, Vector3{1.0, 0.0, 0.0});

    auto t = plane.getIntersection(ray);

    EXPECT_FALSE(t.has_value());
}

TEST(PlaneTest, GetIntersectionBehindRayReturnsNullopt) {
    Plane3 plane(0.0, 0.0, 1.0, -5.0);
    plane.normalize();

    const Ray3 ray(Point3{0.0, 0.0, 10.0}, Vector3{0.0, 0.0, 1.0});

    auto t = plane.getIntersection(ray);

    EXPECT_FALSE(t.has_value());
}

// ------------------------------------------------------------
// reflect()
// ------------------------------------------------------------

TEST(PlaneTest, ReflectOffHorizontalPlane) {
    // Plane z = 0
    Plane3 plane(0.0, 0.0, 1.0, 0.0);
    plane.normalize();

    Ray3 ray(Point3{0.0, 0.0, 1.0}, Vector3{0.0, 0.0, -1.0});

    auto t = plane.getIntersection(ray);
    ASSERT_TRUE(t.has_value());

    plane.reflect(ray, *t);

    // Origin should be at intersection point
    ExpectPointNear(ray.getOrigin(), Point3{0.0, 0.0, 0.0});

    // Direction should be reflected upward
    ExpectVectorNear(ray.getDirection(), Vector3{0.0, 0.0, 1.0});
}

TEST(PlaneTest, ReflectAt45Degrees) {
    // Plane y = 0
    Plane3 plane(0.0, 1.0, 0.0, 0.0);
    plane.normalize();

    Vector3 incoming{1.0, -1.0, 0.0};
    incoming.normalizeInPlace();

    Ray3 ray(Point3{0.0, 1.0, 0.0}, incoming);

    auto t = plane.getIntersection(ray);
    ASSERT_TRUE(t.has_value());

    plane.reflect(ray, *t);

    Vector3 expected{1.0, 1.0, 0.0};
    expected.normalizeInPlace();

    ExpectVectorNear(ray.getDirection(), expected);
}

TEST(PlaneTest, ReflectedDirectionIsNormalized) {
    Plane3 plane(0.0, 0.0, 1.0, 0.0);
    plane.normalize();

    const Vector3 dir{2.0, 0.0, -2.0};
    Ray3 ray(Point3{0.0, 0.0, 1.0}, dir);

    auto t = plane.getIntersection(ray);
    ASSERT_TRUE(t.has_value());

    plane.reflect(ray, *t);

    EXPECT_NEAR(ray.getDirection().magnitude(), 1.0, kEps);
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

    ASSERT_TRUE(line.has_value());

    // Origin should satisfy both planes
    EXPECT_TRUE(IsPointOnPlane(p1, line->getOrigin()));
    EXPECT_TRUE(IsPointOnPlane(p2, line->getOrigin()));

    // Direction should be parallel to z-axis
    EXPECT_TRUE(
        AreVectorsParallel(line->getDirection(), Vector3{0.0, 0.0, 1.0}));
}

TEST(PlaneTest, IntersectPlanesReturnsCorrectLine) {
    // x = 1
    Plane3 p1(1.0, 0.0, 0.0, -1.0);
    // y = 2
    Plane3 p2(0.0, 1.0, 0.0, -2.0);

    p1.normalize();
    p2.normalize();

    auto line = RTB::intersectPlanes(p1, p2);

    ASSERT_TRUE(line.has_value());

    Point3 origin = line->getOrigin();

    ExpectNear(origin[0], 1.0);
    ExpectNear(origin[1], 2.0);

    EXPECT_TRUE(
        AreVectorsParallel(line->getDirection(), Vector3{0.0, 0.0, 1.0}));
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
    ASSERT_TRUE(line.has_value());

    Vector3 expected =
        RTB::crossProduct(p1.getNormalVector(), p2.getNormalVector());
    expected.normalizeInPlace();

    EXPECT_TRUE(AreVectorsEqualOrOpposite(line->getDirection(), expected));
}

// ------------------------------------------------------------
// Regression / Numerical Stability Tests
// ------------------------------------------------------------

TEST(PlaneTest, ThreePointConstructorThenNormalizeProducesUnitNormal) {
    const Point3 a{1.0, 1.0, 1.0};
    const Point3 b{2.0, 1.0, 1.0};
    const Point3 c{1.0, 3.0, 1.0};

    Plane3 plane(a, b, c);
    plane.normalize();

    EXPECT_NEAR(plane.getNormalVector().magnitude(), 1.0, kEps);

    EXPECT_TRUE(IsPointOnPlane(plane, a));
    EXPECT_TRUE(IsPointOnPlane(plane, b));
    EXPECT_TRUE(IsPointOnPlane(plane, c));
}

TEST(PlaneTest, ReflectionPreservesIncidentAngleMagnitude) {
    Plane3 plane(0.0, 0.0, 1.0, 0.0);
    plane.normalize();

    Vector3 dir{1.0, 0.0, -1.0};
    dir.normalizeInPlace();

    Ray3 ray(Point3{0.0, 0.0, 1.0}, dir);

    auto t = plane.getIntersection(ray);
    ASSERT_TRUE(t.has_value());

    plane.reflect(ray, *t);

    EXPECT_NEAR(ray.getDirection().magnitude(), 1.0, kEps);

    Vector3 expected{1.0, 0.0, 1.0};
    expected.normalizeInPlace();

    ExpectVectorNear(ray.getDirection(), expected);
}

}  // namespace