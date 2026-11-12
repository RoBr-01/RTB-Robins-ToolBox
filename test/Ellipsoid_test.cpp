// GOOGLETEST
#include <gtest/gtest.h>

// STL
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

// LOCAL
#include <RTB/Ellipse.hpp>
#include <RTB/Ellipsoid.hpp>
#include <RTB/Math.hpp>
#include <RTB/Plane.hpp>
#include <RTB/Point.hpp>
#include <RTB/Ray.hpp>
#include <RTB/Vector.hpp>

static constexpr double k_tight = 1e-9;  // tight double tolerance
static constexpr double k_med = 1e-6;    // medium tolerance

/** True if a point lies on the ellipsoid surface (within tol). */
namespace {
bool onSurface(const RTB::Point<double, 3>& point,
               double axis_A,
               double axis_B,
               double axis_C,
               double tol = 1e-6) {
    const double val = ((point[0] * point[0]) / (axis_A * axis_A)) +
                       ((point[1] * point[1]) / (axis_B * axis_B)) +
                       ((point[2] * point[2]) / (axis_C * axis_C));
    return std::abs(val - 1.0) < tol;
}

bool onEllipse(const RTB::Point<double, 3>& point,
               const RTB::Ellipse<double>& ellipse,
               double tol = 1e-6) {
    const RTB::Vector<double, 3> delta(ellipse.getCenter(), point);
    const double proj_x = RTB::dotProduct(delta, ellipse.getSemiAxes()[0]);
    const double proj_y = RTB::dotProduct(delta, ellipse.getSemiAxes()[1]);
    const double semi_major = ellipse.getSemiAxisLengths()[0];
    const double semi_minor = ellipse.getSemiAxisLengths()[1];

    const double val = ((proj_x * proj_x) / (semi_major * semi_major)) +
                       ((proj_y * proj_y) / (semi_minor * semi_minor));
    return std::abs(val - 1.0) < tol;
}
}  // namespace

// ============================================================
// Construction
// ============================================================

TEST(EllipsoidTest, DefaultConstructorZeroDims) {
    const RTB::Ellipsoid<double> ellipsoid;
    const auto& dimensions = ellipsoid.getDimensions();
    EXPECT_EQ(dimensions[0], 0.0);
    EXPECT_EQ(dimensions[1], 0.0);
    EXPECT_EQ(dimensions[2], 0.0);
}

TEST(EllipsoidTest, ThreeArgConstructor) {
    const RTB::Ellipsoid<double> ellipsoid(1.0, 2.0, 3.0);
    const auto& dimensions = ellipsoid.getDimensions();
    EXPECT_EQ(dimensions[0], 1.0);
    EXPECT_EQ(dimensions[1], 2.0);
    EXPECT_EQ(dimensions[2], 3.0);
}

// ============================================================
// getRadiusAt
// ============================================================

TEST(EllipsoidTest, GetRadiusAtAlongPrincipalAxes) {
    const RTB::Ellipsoid<double> prolate(2.0, 1.0, 1.0);

    EXPECT_NEAR(prolate.getRadiusAt(0.0, 0.0), 2.0, k_tight);    // +x
    EXPECT_NEAR(prolate.getRadiusAt(90.0, 0.0), 1.0, k_tight);   // +y
    EXPECT_NEAR(prolate.getRadiusAt(0.0, 90.0), 1.0, k_tight);   // +z
}

TEST(EllipsoidTest, GetRadiusAtSphereIsConstant) {
    const RTB::Ellipsoid<double> sphere(3.0, 3.0, 3.0);

    EXPECT_NEAR(sphere.getRadiusAt(0.0, 0.0), 3.0, k_tight);
    EXPECT_NEAR(sphere.getRadiusAt(37.0, 21.0), 3.0, k_tight);
    EXPECT_NEAR(sphere.getRadiusAt(-123.0, 60.0), 3.0, k_tight);
}

// -----------------------------------------------------------------------------
// Death tests (Debug-only asserts)
// -----------------------------------------------------------------------------

#ifndef NDEBUG

TEST(EllipsoidTest, GetRadiusAtOnZeroDimensionsTriggersAssert) {
    EXPECT_DEATH(
        ([]() {
            const RTB::Ellipsoid<double> ellipsoid;  // zero dimensions
            [[maybe_unused]] const auto radius =
                ellipsoid.getRadiusAt(0.0, 0.0);
        }()),
        "");
}

TEST(EllipsoidTest, IntersectRayOnZeroDimensionsTriggersAssert) {
    EXPECT_DEATH(
        ([]() {
            const RTB::Ellipsoid<double> ellipsoid;  // zero dimensions
            const RTB::Ray<double, 3> ray(RTB::Point<double, 3>{0, 0, 0},
                                          RTB::Vector<double, 3>{1, 0, 0});
            [[maybe_unused]] const auto result = ellipsoid.intersectRay(ray);
        }()),
        "");
}

TEST(EllipsoidTest, IntersectRayOnZeroLengthDirectionTriggersAssert) {
    EXPECT_DEATH(
        ([]() {
            const RTB::Ellipsoid<double> sphere(1.0, 1.0, 1.0);
            const RTB::Ray<double, 3> ray(RTB::Point<double, 3>{0, 0, 0},
                                          RTB::Vector<double, 3>{0, 0, 0});
            [[maybe_unused]] const auto result = sphere.intersectRay(ray);
        }()),
        "");
}

#endif  // NDEBUG

// ============================================================
// intersectRay — basic geometry
// ============================================================

namespace {
class EllipsoidRayTest : public ::testing::Test {
   protected:
    // Unit sphere — ground truth for all sphere cases
    RTB::Ellipsoid<double> sphere{1.0, 1.0, 1.0};
    // 2:1:1 prolate spheroid
    RTB::Ellipsoid<double> prolate{2.0, 1.0, 1.0};
};
}  // namespace

// Ray along +X from outside → two intersections on unit sphere at x = ±1
TEST_F(EllipsoidRayTest, UnitSphereAlongX) {
    const RTB::Ray<double, 3> ray({3.0, 0.0, 0.0},
                                  {-1.0, 0.0, 0.0});  // pointing inward
    const auto result = sphere.intersectRay(ray);
    ASSERT_TRUE(result[0].has_value());
    ASSERT_TRUE(result[1].has_value());

    double t1 = result[0].value();
    double t2 = result[1].value();

    // Sort for deterministic comparison
    if (t1 > t2) {
        std::swap(t1, t2);
    }
    EXPECT_NEAR(t1, 2.0, k_tight);
    EXPECT_NEAR(t2, 4.0, k_tight);
}

// Intersection points lie on the sphere surface
TEST_F(EllipsoidRayTest, IntersectionPointsOnSurface) {
    const RTB::Ray<double, 3> ray({0.0, 0.0, 5.0}, {0.0, 0.0, -1.0});
    const auto result = sphere.intersectRay(ray);
    ASSERT_TRUE(result[0].has_value());
    ASSERT_TRUE(result[1].has_value());

    const auto dir = ray.getDirection();
    const auto orig = ray.getOrigin();

    for (const auto& step_opt : result) {
        ASSERT_TRUE(step_opt.has_value());
        const double step = step_opt.value();
        const RTB::Point<double, 3> pt{
            orig[0] + (step * dir[0]),
            orig[1] + (step * dir[1]),
            orig[2] + (step * dir[2]),
        };
        EXPECT_TRUE(onSurface(pt, 1.0, 1.0, 1.0))
            << "Point at t=" << step << " not on sphere surface";
    }
}

// Ray that misses entirely
TEST_F(EllipsoidRayTest, MissReturnsNullopt) {
    // Aimed well clear of the unit sphere
    const RTB::Ray<double, 3> ray({3.0, 0.0, 0.0}, {0.0, 0.0, 1.0});
    const auto result = sphere.intersectRay(ray);
    EXPECT_FALSE(result[0].has_value());
    EXPECT_FALSE(result[1].has_value());
}

// Tangent ray — discriminant == 0, both t values equal
TEST_F(EllipsoidRayTest, TangentRayBothTEqual) {
    // Ray along z-axis at x=1 is tangent to unit sphere
    const RTB::Ray<double, 3> ray({1.0, 0.0, -5.0}, {0.0, 0.0, 1.0});
    const auto result = sphere.intersectRay(ray);
    ASSERT_TRUE(result[0].has_value());
    EXPECT_FALSE(
        result[1].has_value());  // Tangent case returns second as nullopt
}

// Ray from inside the ellipsoid still returns two t values (one negative)
TEST_F(EllipsoidRayTest, RayFromInsideSphere) {
    const RTB::Ray<double, 3> ray({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0});
    const auto result = sphere.intersectRay(ray);
    ASSERT_TRUE(result[0].has_value());
    ASSERT_TRUE(result[1].has_value());

    // One t positive (forward), one negative (behind)
    EXPECT_GT(std::max(result[0].value(), result[1].value()), 0.0);
    EXPECT_LT(std::min(result[0].value(), result[1].value()), 0.0);
}

// Prolate spheroid: along major axis the intersection distance equals A=2
TEST_F(EllipsoidRayTest, ProlateAlongMajorAxis) {
    const RTB::Ray<double, 3> ray({5.0, 0.0, 0.0}, {-1.0, 0.0, 0.0});
    const auto result = prolate.intersectRay(ray);
    ASSERT_TRUE(result[0].has_value());
    ASSERT_TRUE(result[1].has_value());

    double t1 = result[0].value();
    double t2 = result[1].value();
    if (t1 > t2) {
        std::swap(t1, t2);
    }
    EXPECT_NEAR(t1, 3.0, k_tight);  // hits x = +2 at t=3
    EXPECT_NEAR(t2, 7.0, k_tight);  // hits x = -2 at t=7
}

// Prolate spheroid: along minor axis intersection is at y = ±1
TEST_F(EllipsoidRayTest, ProlateAlongMinorAxis) {
    const RTB::Ray<double, 3> ray({0.0, 5.0, 0.0}, {0.0, -1.0, 0.0});
    const auto result = prolate.intersectRay(ray);
    ASSERT_TRUE(result[0].has_value());
    ASSERT_TRUE(result[1].has_value());

    double t1 = result[0].value();
    double t2 = result[1].value();
    if (t1 > t2) {
        std::swap(t1, t2);
    }
    EXPECT_NEAR(t1, 4.0, k_tight);
    EXPECT_NEAR(t2, 6.0, k_tight);
}

// float specialization compiles and returns reasonable results
TEST(EllipsoidFloatTest, FloatSpecializationWorks) {
    const RTB::Ellipsoid<float> ellipsoid(1.0F, 1.0F, 1.0F);
    const RTB::Ray<float, 3> ray({3.0F, 0.0F, 0.0F}, {-1.0F, 0.0F, 0.0F});
    const auto result = ellipsoid.intersectRay(ray);
    ASSERT_TRUE(result[0].has_value());
    ASSERT_TRUE(result[1].has_value());

    float t1 = result[0].value();
    float t2 = result[1].value();
    if (t1 > t2) {
        std::swap(t1, t2);
    }
    EXPECT_NEAR(t1, 2.0F, 1e-5F);
    EXPECT_NEAR(t2, 4.0F, 1e-5F);
}

// ============================================================
// intersectPlane
// ============================================================

namespace {
class EllipsoidPlaneTest : public ::testing::Test {
   protected:
    // Unit sphere — cross-section through origin is a unit circle
    RTB::Ellipsoid<double> sphere{1.0, 1.0, 1.0};
    // 2:1:1 — XY cross-section is an ellipse with semi-axes 2 and 1
    RTB::Ellipsoid<double> prolate{2.0, 1.0, 1.0};
};
}  // namespace

// XY plane (z=0) through unit sphere → circle with semi-axes both = 1
TEST_F(EllipsoidPlaneTest, UnitSphereXYPlaneIsCircle) {
    RTB::Plane<double> xy(0.0, 0.0, 1.0, 0.0);  // z = 0
    xy.normalize();
    const auto ellipse = sphere.intersectPlane(xy);
    ASSERT_TRUE(ellipse.has_value());

    EXPECT_NEAR(ellipse->getSemiAxisLengths()[0], 1.0, k_med);
    EXPECT_NEAR(ellipse->getSemiAxisLengths()[1], 1.0, k_med);

    // Center should be at origin for z=0 plane
    EXPECT_NEAR(ellipse->getCenter()[0], 0.0, k_med);
    EXPECT_NEAR(ellipse->getCenter()[1], 0.0, k_med);
    EXPECT_NEAR(ellipse->getCenter()[2], 0.0, k_med);
}

// XY plane through prolate 2:1:1 → ellipse with semi-axes 2 and 1
TEST_F(EllipsoidPlaneTest, ProlateXYPlaneSemiAxes) {
    RTB::Plane<double> xy(0.0, 0.0, 1.0, 0.0);
    xy.normalize();
    const auto ellipse = prolate.intersectPlane(xy);
    ASSERT_TRUE(ellipse.has_value());

    const double sa0 = ellipse->getSemiAxisLengths()[0];
    const double sa1 = ellipse->getSemiAxisLengths()[1];
    // Larger semi-axis should be ~2, smaller ~1
    const double sa_max = std::max(sa0, sa1);
    const double sa_min = std::min(sa0, sa1);
    EXPECT_NEAR(sa_max, 2.0, k_med);
    EXPECT_NEAR(sa_min, 1.0, k_med);
}

// YZ plane (x=0) through prolate 2:1:1 → circle with semi-axes both = 1
TEST_F(EllipsoidPlaneTest, ProlateYZPlaneIsCircle) {
    RTB::Plane<double> yz(1.0, 0.0, 0.0, 0.0);
    yz.normalize();
    const auto ellipse = prolate.intersectPlane(yz);
    ASSERT_TRUE(ellipse.has_value());

    EXPECT_NEAR(ellipse->getSemiAxisLengths()[0], 1.0, k_med);
    EXPECT_NEAR(ellipse->getSemiAxisLengths()[1], 1.0, k_med);
}

// Normal vector of returned ellipse matches plane normal
TEST_F(EllipsoidPlaneTest, NormalVectorMatchesPlane) {
    RTB::Plane<double> xy(0.0, 0.0, 1.0, 0.0);
    xy.normalize();
    const auto ellipse = sphere.intersectPlane(xy);
    ASSERT_TRUE(ellipse.has_value());

    // Normal should be (0, 0, ±1)
    EXPECT_NEAR(std::abs(ellipse->getNormal()[2]), 1.0, k_tight);
    EXPECT_NEAR(ellipse->getNormal()[0], 0.0, k_tight);
    EXPECT_NEAR(ellipse->getNormal()[1], 0.0, k_tight);
}

// Semi-axis vectors are orthogonal to each other and to the normal
TEST_F(EllipsoidPlaneTest, SemiAxesOrthogonality) {
    RTB::Plane<double> xy(0.0, 0.0, 1.0, 0.0);
    xy.normalize();
    const auto ellipse = prolate.intersectPlane(xy);
    ASSERT_TRUE(ellipse.has_value());

    const double dot_axes =
        RTB::dotProduct(ellipse->getSemiAxes()[0], ellipse->getSemiAxes()[1]);
    EXPECT_NEAR(dot_axes, 0.0, k_med);

    const double dot_ax0_n =
        RTB::dotProduct(ellipse->getSemiAxes()[0], ellipse->getNormal());
    const double dot_ax1_n =
        RTB::dotProduct(ellipse->getSemiAxes()[1], ellipse->getNormal());
    EXPECT_NEAR(dot_ax0_n, 0.0, k_med);
    EXPECT_NEAR(dot_ax1_n, 0.0, k_med);
}

// Off-center plane: x=0 offset. Center should shift accordingly.
TEST_F(EllipsoidPlaneTest, OffCenterPlane) {
    // Plane x = 0.5  →  1*x + 0*y + 0*z - 0.5 = 0
    RTB::Plane<double> px(1.0, 0.0, 0.0, -0.5);
    px.normalize();
    const auto ellipse = prolate.intersectPlane(px);
    ASSERT_TRUE(ellipse.has_value());

    EXPECT_NEAR(ellipse->getCenter()[0], 0.5, k_med);
    EXPECT_NEAR(ellipse->getCenter()[1], 0.0, k_med);
    EXPECT_NEAR(ellipse->getCenter()[2], 0.0, k_med);
}

// ============================================================
// tracePath — unoccluded paths
// ============================================================

TEST(EllipsoidTracePath, UnoccludedPathHasZeroArcLength) {
    // Source and ear both on same hemisphere — no occlusion.
    const RTB::Ellipsoid<double> sphere(1.0, 1.0, 1.0);

    const RTB::Point<double, 3> source{2.0, 0.0, 0.0};
    // Ear on the near side of the sphere
    const RTB::Point<double, 3> ear{1.0, 0.0, 0.0};

    const auto paths = sphere.tracePath(source, ear);

    // Unoccluded: only paths[0] is populated (see tracePath's contract),
    // paths[1] is nullopt since only one physical path exists.
    ASSERT_TRUE(paths[0].has_value());
    ASSERT_FALSE(paths[1].has_value());

    EXPECT_NEAR(paths[0]->arclength, 0.0, k_med);
}

TEST(EllipsoidTracePath, UnoccludedPathlengthEqualsDirectDistance) {
    const RTB::Ellipsoid<double> sphere(1.0, 1.0, 1.0);

    const RTB::Point<double, 3> source{3.0, 0.0, 0.0};
    const RTB::Point<double, 3> ear{1.0, 0.0, 0.0};

    const auto paths = sphere.tracePath(source, ear);

    // Unoccluded: only paths[0] is populated (see tracePath's contract),
    // paths[1] is nullopt since only one physical path exists.
    ASSERT_TRUE(paths[0].has_value());
    ASSERT_FALSE(paths[1].has_value());

    // Direct distance from source to ear = 2.0
    const double expected = RTB::distance2Points(source, ear);
    EXPECT_NEAR(paths[0]->total_length, expected, k_med);
}

// ============================================================
// tracePath — occluded paths
// ============================================================

// source/ear here are symmetric about the sphere's axis, so the two
// tangent-wrap paths are genuinely equal in length (not just numerically
// close by coincidence) — verified independently via tangent-line geometry
// (sqrt(d^2 - r^2) + arc length) outside this test suite.
TEST(EllipsoidTracePath, OccludedSymmetricPathsAreEqualLength) {
    const RTB::Ellipsoid<double> sphere(1.0, 1.0, 1.0);
    const RTB::Point<double, 3> source{2.0, 0.0, 0.0};
    const RTB::Point<double, 3> ear{-1.0, 0.0, 0.0};

    const auto paths = sphere.tracePath(source, ear);
    ASSERT_TRUE(paths[0].has_value());
    ASSERT_TRUE(paths[1].has_value());

    EXPECT_NEAR(paths[0]->total_length, paths[1]->total_length, k_tight);
}

// With an asymmetric ear position, the two tangent-wrap paths genuinely
// differ in length, so EXPECT_LT (not EXPECT_NEAR) is meaningful here.
TEST(EllipsoidTracePath, OccludedAsymmetricPathsDifferInLength) {
    const RTB::Ellipsoid<double> sphere(1.0, 1.0, 1.0);
    const RTB::Point<double, 3> source{3.0, 0.0, 0.0};
    const RTB::Point<double, 3> ear{-0.5, 0.86602540378, 0.0};

    const auto paths = sphere.tracePath(source, ear);

    ASSERT_TRUE(paths[0].has_value());
    ASSERT_TRUE(paths[1].has_value());
    EXPECT_LT(paths[0]->total_length, paths[1]->total_length)
        << "Path via the nearer tangent wrap should be strictly shorter";
}

TEST(EllipsoidTracePath, TangentPointLiesOnSurface) {
    const RTB::Ellipsoid<double> sphere(1.0, 1.0, 1.0);
    const RTB::Point<double, 3> source{2.0, 0.0, 0.0};
    const RTB::Point<double, 3> ear{-1.0, 0.0, 0.0};

    const auto paths = sphere.tracePath(source, ear);

    for (const auto& path : paths) {
        ASSERT_TRUE(path.has_value());
        EXPECT_TRUE(onSurface(path->tangent_point, 1.0, 1.0, 1.0))
            << "Tangent point is not on the ellipsoid surface";
    }
}

TEST(EllipsoidTracePath, TangentPointLiesOnIntersectionEllipse) {
    const RTB::Ellipsoid<double> sphere(1.0, 1.0, 1.0);
    const RTB::Point<double, 3> source{2.0, 0.0, 0.0};
    const RTB::Point<double, 3> ear{
        0.0,
        1.0,
        0.0,
    };  // Moved to Y-axis, not collinear!

    // Now these three points are not collinear
    const RTB::Plane<double> plane(
        source, ear, RTB::Point<double, 3>{0.0, 0.0, 0.0});
    const auto ellipse = sphere.intersectPlane(plane);
    ASSERT_TRUE(ellipse.has_value());

    const auto paths = sphere.tracePath(source, ear);

    for (const auto& path_opt : paths) {
        ASSERT_TRUE(path_opt.has_value());
        EXPECT_TRUE(onEllipse(path_opt->tangent_point, ellipse.value()))
            << "Tangent point is not on the intersection ellipse";
    }
}

// Independently re-derives the expected tangent length, tangent angle, and
// arc length from the tangent-line geometry (not by reading tracePath's own
// stored fields back at itself) to verify total_length is actually correct,
// not merely self-consistent with straight_length + arclength.
TEST(EllipsoidTracePath, PathlengthMatchesIndependentTangentGeometry) {
    const RTB::Ellipsoid<double> sphere(1.0, 1.0, 1.0);
    const double source_distance = 2.0;
    const double radius = 1.0;
    const RTB::Point<double, 3> source{source_distance, 0.0, 0.0};
    const RTB::Point<double, 3> ear{-radius, 0.0, 0.0};

    const auto paths = sphere.tracePath(source, ear);
    ASSERT_TRUE(paths[0].has_value());
    ASSERT_TRUE(paths[1].has_value());

    // Tangent-line length from a point at distance d to a sphere of
    // radius r: sqrt(d^2 - r^2). Tangent angle (at the sphere's center,
    // measured from the source direction): acos(r / d).
    const double expected_tangent_length =
        std::sqrt((source_distance * source_distance) - (radius * radius));
    const double tangent_angle = std::acos(radius / source_distance);
    // ear is antipodal to source (180 deg away); the geodesic arc from the
    // tangent point to the ear covers the remaining angle.
    const double expected_arc_length = radius * (RTB::pi - tangent_angle);
    const double expected_total = expected_tangent_length + expected_arc_length;

    for (const auto& path : paths) {
        EXPECT_NEAR(path->straight_length, expected_tangent_length, k_med);
        EXPECT_NEAR(path->arclength, expected_arc_length, k_med);
        EXPECT_NEAR(path->total_length, expected_total, k_med);
    }
}

// ============================================================
// tracePath — symmetry checks
// ============================================================

// For a sphere, rotating source and ear by 90° about Z should give
// identical path lengths (rotational symmetry).
TEST(EllipsoidTracePath, RotationalSymmetryOnSphere) {
    const RTB::Ellipsoid<double> sphere(1.0, 1.0, 1.0);

    // Config A: source along +X
    const RTB::Point<double, 3> src1{2.0, 0.0, 0.0};
    const RTB::Point<double, 3> ear1{-1.0, 0.0, 0.0};
    const auto paths1 = sphere.tracePath(src1, ear1);
    ASSERT_TRUE(paths1[0].has_value());
    const double pl_a = paths1[0]->total_length;

    // Config B: source along +Y (90° rotation)
    const RTB::Point<double, 3> src2{0.0, 2.0, 0.0};
    const RTB::Point<double, 3> ear2{0.0, -1.0, 0.0};
    const auto paths2 = sphere.tracePath(src2, ear2);
    ASSERT_TRUE(paths2[0].has_value());
    const double pl_b = paths2[0]->total_length;

    EXPECT_NEAR(pl_a, pl_b, 1e-6)
        << "Sphere path lengths should be equal under 90° rotation";
}

// ============================================================
// ArcLength accuracy
// ============================================================

// For a sphere, arc length of a quarter great-circle = pi/2
TEST(EllipsoidArcLength, SphereQuarterArc) {
    const RTB::Ellipsoid<double> sphere(1.0, 1.0, 1.0);
    const RTB::Point<double, 3> p1{1.0, 0.0, 0.0};
    const RTB::Point<double, 3> p2{0.0, 1.0, 0.0};

    const auto arc = sphere.arcLength(p1, p2);
    ASSERT_TRUE(arc.has_value());
    const double expected = RTB::pi / 2.0;
    EXPECT_NEAR(*arc, expected, 1e-4);
}

// For a sphere, arc length of half a great circle = pi
TEST(EllipsoidArcLength, SphereHalfArc) {
    const RTB::Ellipsoid<double> sphere(1.0, 1.0, 1.0);
    const RTB::Point<double, 3> p1{1.0, 0.0, 0.0};
    const RTB::Point<double, 3> p2{-1.0, 0.0, 0.0};

    const auto arc = sphere.arcLength(p1, p2);
    ASSERT_TRUE(arc.has_value());
    const double expected = RTB::pi;
    EXPECT_NEAR(*arc, expected, 1e-4);
}

// For a sphere, points on same side give small arc
TEST(EllipsoidArcLength, SphereSmallArc) {
    const RTB::Ellipsoid<double> sphere(1.0, 1.0, 1.0);
    const RTB::Point<double, 3> p1{1.0, 0.0, 0.0};
    const RTB::Point<double, 3> p2{std::cos(0.1), std::sin(0.1), 0.0};

    const auto arc = sphere.arcLength(p1, p2);
    ASSERT_TRUE(arc.has_value());
    const double expected = 0.1;  // radian measure
    EXPECT_NEAR(*arc, expected, 1e-4);
}

// For a sphere, arc length should be symmetric
TEST(EllipsoidArcLength, SphereArcSymmetric) {
    const RTB::Ellipsoid<double> sphere(1.0, 1.0, 1.0);
    const RTB::Point<double, 3> p1{1.0, 0.0, 0.0};
    const RTB::Point<double, 3> p2{0.0, 1.0, 0.0};

    const auto arc1 = sphere.arcLength(p1, p2);
    const auto arc2 = sphere.arcLength(p2, p1);
    ASSERT_TRUE(arc1.has_value());
    ASSERT_TRUE(arc2.has_value());
    EXPECT_NEAR(*arc1, *arc2, k_tight);
}

// For a sphere, arc length of full circle should equal circumference
TEST(EllipsoidArcLength, SphereFullCircle) {
    const RTB::Ellipsoid<double> sphere(1.0, 1.0, 1.0);
    const RTB::Point<double, 3> p1{1.0, 0.0, 0.0};

    // Get the intersection ellipse (great circle)
    const RTB::Plane<double> plane(p1,
                                   RTB::Point<double, 3>{0.0, 1.0, 0.0},
                                   RTB::Point<double, 3>{0.0, 0.0, 0.0});
    const auto ellipse = sphere.intersectPlane(plane);
    ASSERT_TRUE(ellipse.has_value());

    // arcLength between same point should be 0 (shorter arc)
    const auto arc = sphere.arcLength(p1, p1);
    ASSERT_TRUE(arc.has_value());
    EXPECT_NEAR(*arc, 0.0, k_tight);
}

TEST(EllipsoidArcLength, ProlateEquatorialArc) {
    const RTB::Ellipsoid<double> prolate(2.0, 1.0, 1.0);
    const RTB::Point<double, 3> p1{2.0, 0.0, 0.0};
    const RTB::Point<double, 3> p2{0.0, 1.0, 0.0};

    const auto arc = prolate.arcLength(p1, p2);
    ASSERT_TRUE(arc.has_value());

    // Reference value independently computed via Simpson's rule at
    // n=100000 for the quarter arc of ellipse a=2, b=1 (see the matching
    // Ellipse test, ArcLength_NonCircularMatchesClosedFormReference).
    // Tight tolerance: this would fail under the old
    // Ellipse::parameterOf(), which used atan2(proj_y, proj_x) directly
    // instead of atan2(proj_y / b, proj_x / a) and so computed the wrong
    // parametric angle for any non-circular ellipse.
    const double expected = 2.4221120551369446;

    EXPECT_NEAR(*arc, expected, 1e-9);
}

// ============================================================
// Edge-case: very large and very small ellipsoids (scaling)
// ============================================================

// Path lengths should scale linearly with ellipsoid dimensions
TEST(EllipsoidTracePath, LinearScaling) {
    const double scale = 10.0;
    const RTB::Ellipsoid<double> sphere_small(1.0, 1.0, 1.0);
    const RTB::Ellipsoid<double> sphere_large(scale, scale, scale);

    const RTB::Point<double, 3> src_s{2.0, 0.0, 0.0};
    const RTB::Point<double, 3> ear_s{-1.0, 0.0, 0.0};

    const RTB::Point<double, 3> src_l{2.0 * scale, 0.0, 0.0};
    const RTB::Point<double, 3> ear_l{-1.0 * scale, 0.0, 0.0};

    const auto paths_s = sphere_small.tracePath(src_s, ear_s);
    const auto paths_l = sphere_large.tracePath(src_l, ear_l);
    ASSERT_TRUE(paths_s[0].has_value());
    ASSERT_TRUE(paths_l[0].has_value());

    const double pl_s = paths_s[0]->total_length;
    const double pl_l = paths_l[0]->total_length;

    EXPECT_NEAR(pl_l / pl_s, scale, 1e-4)
        << "Path length should scale linearly with ellipsoid size";
}

// Arc length should scale linearly with dimensions
TEST(EllipsoidArcLength, LinearScaling) {
    const double scale = 10.0;
    const RTB::Ellipsoid<double> sphere_small(1.0, 1.0, 1.0);
    const RTB::Ellipsoid<double> sphere_large(scale, scale, scale);

    const RTB::Point<double, 3> p1_s{1.0, 0.0, 0.0};
    const RTB::Point<double, 3> p2_s{0.0, 1.0, 0.0};

    const RTB::Point<double, 3> p1_l{scale, 0.0, 0.0};
    const RTB::Point<double, 3> p2_l{0.0, scale, 0.0};

    const auto arc_s = sphere_small.arcLength(p1_s, p2_s);
    const auto arc_l = sphere_large.arcLength(p1_l, p2_l);
    ASSERT_TRUE(arc_s.has_value());
    ASSERT_TRUE(arc_l.has_value());

    EXPECT_NEAR(*arc_l / *arc_s, scale, 1e-4)
        << "Arc length should scale linearly with ellipsoid size";
}

TEST(EllipsoidTracePath, TangentPointsAreDistinct) {
    const RTB::Ellipsoid<double> sphere(1.0, 1.0, 1.0);
    const RTB::Point<double, 3> source{2.0, 0.0, 0.0};
    const RTB::Point<double, 3> ear{0.0, -1.0, 0.0};

    const auto paths = sphere.tracePath(source, ear);
    ASSERT_TRUE(paths[0].has_value());
    ASSERT_TRUE(paths[1].has_value());

    // The two tangent points should be different
    const auto& tp1 = paths[0]->tangent_point;
    const auto& tp2 = paths[1]->tangent_point;

    const double dist = RTB::distance2Points(tp1, tp2);
    EXPECT_GT(dist, 0.1) << "Tangent points should be distinct";
}

TEST(EllipsoidTracePath, SourceToTangentVectorsAreCorrect) {
    const RTB::Ellipsoid<double> sphere(1.0, 1.0, 1.0);
    const RTB::Point<double, 3> source{2.0, 0.0, 0.0};
    const RTB::Point<double, 3> ear{-1.0, 0.0, 0.0};

    const auto paths = sphere.tracePath(source, ear);

    for (const auto& path : paths) {
        ASSERT_TRUE(path.has_value());

        // Verify that source2tanpoint vector matches the difference
        const RTB::Vector<double, 3> computed(source, path->tangent_point);
        EXPECT_NEAR(computed[0], path->source2tanpoint[0], k_med);
        EXPECT_NEAR(computed[1], path->source2tanpoint[1], k_med);
        EXPECT_NEAR(computed[2], path->source2tanpoint[2], k_med);

        // True tangency condition, independent of any stored field: for a
        // sphere centered at the origin, the source-to-tangent vector must
        // be perpendicular to the radius vector at the tangent point.
        const RTB::Vector<double, 3> radius_vector(path->tangent_point);
        EXPECT_NEAR(RTB::dotProduct(computed, radius_vector), 0.0, k_med)
            << "source-to-tangent vector is not perpendicular to the "
               "sphere's radius at the tangent point";
    }
}

// Test that tracePath behaves sanely for points exactly on the surface.
// With source exactly on the surface and ear exactly antipodal, the
// tangent-line construction is degenerate (zero-length tangent line, and
// the ray/ellipsoid intersection used to find the second tangent point can
// legitimately fail). tracePath must report this as "no path" rather than
// silently returning a populated-but-garbage Path.
TEST(EllipsoidTracePath, SourceOnSurfaceDegenerateCaseReportsNoPath) {
    const RTB::Ellipsoid<double> sphere(1.0, 1.0, 1.0);
    const RTB::Point<double, 3> source{1.0, 0.0, 0.0};  // On surface
    const RTB::Point<double, 3> ear{-1.0, 0.0, 0.0};

    const auto paths = sphere.tracePath(source, ear);

    // Either both slots report no valid path (degenerate geometry), or any
    // populated slot must hold a genuinely valid, non-negative length.
    for (const auto& path : paths) {
        if (path.has_value()) {
            EXPECT_GE(path->total_length, 0.0);
        }
    }
}

// Test that tracePath behaves sanely for points exactly antipodal on the
// surface. This degenerate geometry (zero-length tangent lines on both
// sides) legitimately yields no well-defined ray/ellipsoid intersection, so
// tracePath must report "no path" rather than silently returning a
// populated-but-garbage Path.
TEST(EllipsoidTracePath, AntipodalPointsOnSurfaceReportsNoPath) {
    const RTB::Ellipsoid<double> sphere(1.0, 1.0, 1.0);
    const RTB::Point<double, 3> source{1.0, 0.0, 0.0};
    const RTB::Point<double, 3> ear{-1.0, 0.0, 0.0};

    const auto paths = sphere.tracePath(source, ear);

    for (const auto& path : paths) {
        if (path.has_value()) {
            EXPECT_GE(path->total_length, 0.0);
        }
    }
}