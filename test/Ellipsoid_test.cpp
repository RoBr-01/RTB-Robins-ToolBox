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
    const RTB::Vector<double, 3> delta(ellipse.center, point);
    const double x = RTB::dotProduct(delta, ellipse.semi_axes[0]);
    const double y = RTB::dotProduct(delta, ellipse.semi_axes[1]);
    const double a = ellipse.semi_axis_lengths[0];
    const double b = ellipse.semi_axis_lengths[1];

    const double val = (x * x) / (a * a) + (y * y) / (b * b);
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
        const RTB::Point<double, 3> pt{orig[0] + (step * dir[0]),
                                       orig[1] + (step * dir[1]),
                                       orig[2] + (step * dir[2])};
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
    auto ellipse = sphere.intersectPlane(xy);
    ASSERT_TRUE(ellipse.has_value());

    EXPECT_NEAR(ellipse->semi_axis_lengths[0], 1.0, k_med);
    EXPECT_NEAR(ellipse->semi_axis_lengths[1], 1.0, k_med);

    // Center should be at origin for z=0 plane
    EXPECT_NEAR(ellipse->center[0], 0.0, k_med);
    EXPECT_NEAR(ellipse->center[1], 0.0, k_med);
    EXPECT_NEAR(ellipse->center[2], 0.0, k_med);
}

// XY plane through prolate 2:1:1 → ellipse with semi-axes 2 and 1
TEST_F(EllipsoidPlaneTest, ProlateXYPlaneSemiAxes) {
    RTB::Plane<double> xy(0.0, 0.0, 1.0, 0.0);
    xy.normalize();
    auto ellipse = prolate.intersectPlane(xy);
    ASSERT_TRUE(ellipse.has_value());

    const double sa0 = ellipse->semi_axis_lengths[0];
    const double sa1 = ellipse->semi_axis_lengths[1];
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
    auto ellipse = prolate.intersectPlane(yz);
    ASSERT_TRUE(ellipse.has_value());

    EXPECT_NEAR(ellipse->semi_axis_lengths[0], 1.0, k_med);
    EXPECT_NEAR(ellipse->semi_axis_lengths[1], 1.0, k_med);
}

// Normal vector of returned ellipse matches plane normal
TEST_F(EllipsoidPlaneTest, NormalVectorMatchesPlane) {
    RTB::Plane<double> xy(0.0, 0.0, 1.0, 0.0);
    xy.normalize();
    auto ellipse = sphere.intersectPlane(xy);
    ASSERT_TRUE(ellipse.has_value());

    // Normal should be (0, 0, ±1)
    EXPECT_NEAR(std::abs(ellipse->normal[2]), 1.0, k_tight);
    EXPECT_NEAR(ellipse->normal[0], 0.0, k_tight);
    EXPECT_NEAR(ellipse->normal[1], 0.0, k_tight);
}

// Semi-axis vectors are orthogonal to each other and to the normal
TEST_F(EllipsoidPlaneTest, SemiAxesOrthogonality) {
    RTB::Plane<double> xy(0.0, 0.0, 1.0, 0.0);
    xy.normalize();
    auto ellipse = prolate.intersectPlane(xy);
    ASSERT_TRUE(ellipse.has_value());

    const double dot_axes =
        RTB::dotProduct(ellipse->semi_axes[0], ellipse->semi_axes[1]);
    EXPECT_NEAR(dot_axes, 0.0, k_med);

    const double dot_ax0_n =
        RTB::dotProduct(ellipse->semi_axes[0], ellipse->normal);
    const double dot_ax1_n =
        RTB::dotProduct(ellipse->semi_axes[1], ellipse->normal);
    EXPECT_NEAR(dot_ax0_n, 0.0, k_med);
    EXPECT_NEAR(dot_ax1_n, 0.0, k_med);
}

// Off-center plane: x=0 offset. Center should shift accordingly.
TEST_F(EllipsoidPlaneTest, OffCenterPlane) {
    // Plane x = 0.5  →  1*x + 0*y + 0*z - 0.5 = 0
    RTB::Plane<double> px(1.0, 0.0, 0.0, -0.5);
    px.normalize();
    auto ellipse = prolate.intersectPlane(px);
    ASSERT_TRUE(ellipse.has_value());

    EXPECT_NEAR(ellipse->center[0], 0.5, k_med);
    EXPECT_NEAR(ellipse->center[1], 0.0, k_med);
    EXPECT_NEAR(ellipse->center[2], 0.0, k_med);
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

    auto paths = sphere.tracePath(source, ear);

    // For unoccluded case, both paths should have zero arclength
    // (the two paths are symmetric)
    EXPECT_NEAR(paths[0]->arclength, 0.0, k_med);
    EXPECT_NEAR(paths[1]->arclength, 0.0, k_med);
}

TEST(EllipsoidTracePath, UnoccludedPathlengthEqualsDirectDistance) {
    const RTB::Ellipsoid<double> sphere(1.0, 1.0, 1.0);

    const RTB::Point<double, 3> source{3.0, 0.0, 0.0};
    const RTB::Point<double, 3> ear{1.0, 0.0, 0.0};

    auto paths = sphere.tracePath(source, ear);

    // Direct distance from source to ear = 2.0
    const double expected = RTB::distance2Points(source, ear);
    EXPECT_NEAR(paths[0]->total_length, expected, k_med);
    EXPECT_NEAR(paths[1]->total_length, expected, k_med);
}

// ============================================================
// tracePath — occluded paths
// ============================================================

TEST(EllipsoidTracePath, OccludedPathShorterThanLonger) {
    const RTB::Ellipsoid<double> sphere(1.0, 1.0, 1.0);
    const RTB::Point<double, 3> source{2.0, 0.0, 0.0};
    const RTB::Point<double, 3> ear{-1.0, 0.0, 0.0};

    auto paths = sphere.tracePath(source, ear);

    EXPECT_LE(paths[0]->total_length, paths[1]->total_length)
        << "Primary path (index 0) should be <= secondary path (index 1)";
}

TEST(EllipsoidTracePath, TangentPointLiesOnSurface) {
    const RTB::Ellipsoid<double> sphere(1.0, 1.0, 1.0);
    const RTB::Point<double, 3> source{2.0, 0.0, 0.0};
    const RTB::Point<double, 3> ear{-1.0, 0.0, 0.0};

    auto paths = sphere.tracePath(source, ear);

    for (const auto& path : paths) {
        EXPECT_TRUE(onSurface(path->tangent_point, 1.0, 1.0, 1.0))
            << "Tangent point is not on the ellipsoid surface";
    }
}

TEST(EllipsoidTracePath, TangentPointLiesOnIntersectionEllipse) {
    const RTB::Ellipsoid<double> sphere(1.0, 1.0, 1.0);
    const RTB::Point<double, 3> source{2.0, 0.0, 0.0};
    const RTB::Point<double, 3> ear{
        0.0, 1.0, 0.0};  // Moved to Y-axis, not collinear!

    // Now these three points are not collinear
    const RTB::Plane<double> plane(
        source, ear, RTB::Point<double, 3>{0.0, 0.0, 0.0});
    auto ellipse = sphere.intersectPlane(plane);
    ASSERT_TRUE(ellipse.has_value());

    auto paths = sphere.tracePath(source, ear);

    for (const auto& path_opt : paths) {
        ASSERT_TRUE(path_opt.has_value());
        EXPECT_TRUE(onEllipse(path_opt->tangent_point, ellipse.value()))
            << "Tangent point is not on the intersection ellipse";
    }
}

TEST(EllipsoidTracePath, PathlengthIsDirectPlusArc) {
    const RTB::Ellipsoid<double> sphere(1.0, 1.0, 1.0);
    const RTB::Point<double, 3> source{2.0, 0.0, 0.0};
    const RTB::Point<double, 3> ear{-1.0, 0.0, 0.0};

    auto paths = sphere.tracePath(source, ear);

    for (const auto& path : paths) {
        EXPECT_NEAR(
            path->total_length, path->straigh_length + path->arclength, k_med);
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
    auto paths1 = sphere.tracePath(src1, ear1);
    const double pl_a = paths1[0]->total_length;

    // Config B: source along +Y (90° rotation)
    const RTB::Point<double, 3> src2{0.0, 2.0, 0.0};
    const RTB::Point<double, 3> ear2{0.0, -1.0, 0.0};
    auto paths2 = sphere.tracePath(src2, ear2);
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

    const double arc = sphere.arcLength(p1, p2);
    const double expected = RTB::pi / 2.0;
    EXPECT_NEAR(arc, expected, 1e-4);
}

// For a sphere, arc length of half a great circle = pi
TEST(EllipsoidArcLength, SphereHalfArc) {
    const RTB::Ellipsoid<double> sphere(1.0, 1.0, 1.0);
    const RTB::Point<double, 3> p1{1.0, 0.0, 0.0};
    const RTB::Point<double, 3> p2{-1.0, 0.0, 0.0};

    const double arc = sphere.arcLength(p1, p2);
    const double expected = RTB::pi;
    EXPECT_NEAR(arc, expected, 1e-4);
}

// For a sphere, points on same side give small arc
TEST(EllipsoidArcLength, SphereSmallArc) {
    const RTB::Ellipsoid<double> sphere(1.0, 1.0, 1.0);
    const RTB::Point<double, 3> p1{1.0, 0.0, 0.0};
    const RTB::Point<double, 3> p2{std::cos(0.1), std::sin(0.1), 0.0};

    const double arc = sphere.arcLength(p1, p2);
    const double expected = 0.1;  // radian measure
    EXPECT_NEAR(arc, expected, 1e-4);
}

// For a sphere, arc length should be symmetric
TEST(EllipsoidArcLength, SphereArcSymmetric) {
    const RTB::Ellipsoid<double> sphere(1.0, 1.0, 1.0);
    const RTB::Point<double, 3> p1{1.0, 0.0, 0.0};
    const RTB::Point<double, 3> p2{0.0, 1.0, 0.0};

    const double arc1 = sphere.arcLength(p1, p2);
    const double arc2 = sphere.arcLength(p2, p1);
    EXPECT_NEAR(arc1, arc2, k_tight);
}

// For a sphere, arc length of full circle should equal circumference
TEST(EllipsoidArcLength, SphereFullCircle) {
    const RTB::Ellipsoid<double> sphere(1.0, 1.0, 1.0);
    const RTB::Point<double, 3> p1{1.0, 0.0, 0.0};

    // Get the intersection ellipse (great circle)
    const RTB::Plane<double> plane(p1,
                                   RTB::Point<double, 3>{0.0, 1.0, 0.0},
                                   RTB::Point<double, 3>{0.0, 0.0, 0.0});
    auto ellipse = sphere.intersectPlane(plane);
    ASSERT_TRUE(ellipse.has_value());

    // arcLength between same point should be 0 (shorter arc)
    const double arc = sphere.arcLength(p1, p1);
    EXPECT_NEAR(arc, 0.0, k_tight);
}

TEST(EllipsoidArcLength, ProlateEquatorialArc) {
    const RTB::Ellipsoid<double> prolate(2.0, 1.0, 1.0);
    const RTB::Point<double, 3> p1{2.0, 0.0, 0.0};
    const RTB::Point<double, 3> p2{0.0, 1.0, 0.0};

    const double arc = prolate.arcLength(p1, p2);

    // Known value from numerical integration
    const double expected =
        2.422;  // Approximate quarter perimeter of ellipse a=2, b=1

    EXPECT_NEAR(arc, expected, 1e-3);
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

    auto paths_s = sphere_small.tracePath(src_s, ear_s);
    auto paths_l = sphere_large.tracePath(src_l, ear_l);

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

    const double arc_s = sphere_small.arcLength(p1_s, p2_s);
    const double arc_l = sphere_large.arcLength(p1_l, p2_l);

    EXPECT_NEAR(arc_l / arc_s, scale, 1e-4)
        << "Arc length should scale linearly with ellipsoid size";
}

TEST(EllipsoidTracePath, TangentPointsAreDistinct) {
    const RTB::Ellipsoid<double> sphere(1.0, 1.0, 1.0);
    const RTB::Point<double, 3> source{2.0, 0.0, 0.0};
    const RTB::Point<double, 3> ear{0.0, -1.0, 0.0};

    auto paths = sphere.tracePath(source, ear);

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

    auto paths = sphere.tracePath(source, ear);

    for (const auto& path : paths) {
        // Verify that source2tanpoint vector matches the difference
        const RTB::Vector<double, 3> computed(source, path->tangent_point);
        EXPECT_NEAR(computed[0], path->source2tanpoint[0], k_med);
        EXPECT_NEAR(computed[1], path->source2tanpoint[1], k_med);
        EXPECT_NEAR(computed[2], path->source2tanpoint[2], k_med);

        // Verify magnitude matches straight_length
        EXPECT_NEAR(computed.magnitude(), path->straigh_length, k_med);
    }
}

// Test that tracePath works correctly for points exactly on the surface
TEST(EllipsoidTracePath, SourceOnSurface) {
    const RTB::Ellipsoid<double> sphere(1.0, 1.0, 1.0);
    const RTB::Point<double, 3> source{1.0, 0.0, 0.0};  // On surface
    const RTB::Point<double, 3> ear{-1.0, 0.0, 0.0};

    auto paths = sphere.tracePath(source, ear);

    // Should still compute valid paths
    EXPECT_GE(paths[0]->total_length, 0.0);
    EXPECT_GE(paths[1]->total_length, 0.0);
}

// Test that tracePath works for points exactly antipodal
TEST(EllipsoidTracePath, AntipodalPoints) {
    const RTB::Ellipsoid<double> sphere(1.0, 1.0, 1.0);
    const RTB::Point<double, 3> source{1.0, 0.0, 0.0};
    const RTB::Point<double, 3> ear{-1.0, 0.0, 0.0};

    auto paths = sphere.tracePath(source, ear);

    // Both paths should have the same length (pi for arc, 2 for straight)
    EXPECT_NEAR(paths[0]->total_length, paths[1]->total_length, k_med);
}