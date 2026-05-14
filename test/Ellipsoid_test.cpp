// GOOGLETEST
#include <gtest/gtest.h>

// STL
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

// LOCAL
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

TEST(EllipsoidTest, InitializerListConstructor) {
    const RTB::Ellipsoid<double> ellipsoid({4.0, 5.0, 6.0});
    const auto& dimensions = ellipsoid.getDimensions();
    EXPECT_EQ(dimensions[0], 4.0);
    EXPECT_EQ(dimensions[1], 5.0);
    EXPECT_EQ(dimensions[2], 6.0);
}

TEST(EllipsoidTest, ArrayConstructor) {
    const std::array<double, 3> dims = {7.0, 8.0, 9.0};
    const RTB::Ellipsoid<double> ellipsoid(dims);
    EXPECT_EQ(ellipsoid.getDimensions(), dims);
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
    ASSERT_TRUE(result.has_value());

    // t values should give intersection at x = 1 (t=2) and x = -1 (t=4)
    double t1{};
    double t2{};
    if (result.has_value()) {
        const auto& hits = *result;
        t1 = hits.at(0);
        t2 = hits.at(1);
    }

    // Sort for deterministic comparison
    if (t1 > t2) {
        std::swap(t1, t2);
    }
    EXPECT_NEAR(t1, 2.0, k_tight);
    EXPECT_NEAR(t2, 4.0, k_tight);
}

// Intersection points lie on the sphere surface
TEST_F(EllipsoidRayTest, IntersectionPointsOnSurface) {
    const RTB::Ray<double, 3> ray({0.0, 0.0, 5.0}, {0.0, 0.3, -1.0});
    const auto result = sphere.intersectRay(ray);
    ASSERT_TRUE(result.has_value());

    const auto dir = ray.getDirection();
    const auto orig = ray.getOrigin();

    if (result.has_value()) {
        for (const double step : *result) {
            const RTB::Point<double, 3> pt{orig[0] + (step * dir[0]),
                                           orig[1] + (step * dir[1]),
                                           orig[2] + (step * dir[2])};
            EXPECT_TRUE(onSurface(pt, 1.0, 1.0, 1.0))
                << "Point at t=" << step << " not on sphere surface";
        }
    }
}

// Ray that misses entirely
TEST_F(EllipsoidRayTest, MissReturnsNullopt) {
    // Aimed well clear of the unit sphere
    const RTB::Ray<double, 3> ray({3.0, 0.0, 0.0}, {0.0, 0.0, 1.0});
    EXPECT_FALSE(sphere.intersectRay(ray).has_value());
}

// Tangent ray — discriminant == 0, both t values equal
TEST_F(EllipsoidRayTest, TangentRayBothTEqual) {
    // Ray along z-axis at x=1 is tangent to unit sphere
    const RTB::Ray<double, 3> ray({1.0, 0.0, -5.0}, {0.0, 0.0, 1.0});
    const auto result = sphere.intersectRay(ray);
    ASSERT_TRUE(result.has_value());

    if (result.has_value()) {
        const auto& hits = *result;
        EXPECT_NEAR(hits.at(0), hits.at(1), 1e-6);
    }
}

// Ray from inside the ellipsoid still returns two t values (one negative)
TEST_F(EllipsoidRayTest, RayFromInsideSphere) {
    const RTB::Ray<double, 3> ray({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0});
    const auto result = sphere.intersectRay(ray);
    ASSERT_TRUE(result.has_value());
    if (result.has_value()) {
        const auto& hits = *result;
        // One t positive (forward), one negative (behind)
        EXPECT_GT(std::max(hits.at(0), hits.at(1)), 0.0);
        EXPECT_LT(std::min(hits.at(0), hits.at(1)), 0.0);
    }
}

// Prolate spheroid: along major axis the intersection distance equals A=2
TEST_F(EllipsoidRayTest, ProlateAlongMajorAxis) {
    const RTB::Ray<double, 3> ray({5.0, 0.0, 0.0}, {-1.0, 0.0, 0.0});
    const auto result = prolate.intersectRay(ray);
    ASSERT_TRUE(result.has_value());

    if (result.has_value()) {
        const auto& hits = *result;
        double t1 = hits.at(0);
        double t2 = hits.at(1);
        if (t1 > t2) {
            std::swap(t1, t2);
        }
        EXPECT_NEAR(t1, 3.0, k_tight);  // hits x = +2 at t=3
        EXPECT_NEAR(t2, 7.0, k_tight);  // hits x = -2 at t=7
    }
}

// Prolate spheroid: along minor axis intersection is at y = ±1
TEST_F(EllipsoidRayTest, ProlateAlongMinorAxis) {
    const RTB::Ray<double, 3> ray({0.0, 5.0, 0.0}, {0.0, -1.0, 0.0});
    const auto result = prolate.intersectRay(ray);
    ASSERT_TRUE(result.has_value());

    if (result.has_value()) {
        const auto& hits = *result;
        double t1 = hits.at(0);
        double t2 = hits.at(1);
        if (t1 > t2) {
            std::swap(t1, t2);
        }
        EXPECT_NEAR(t1, 4.0, k_tight);
        EXPECT_NEAR(t2, 6.0, k_tight);
    }
}

// float specialization compiles and returns reasonable results
TEST(EllipsoidFloatTest, FloatSpecializationWorks) {
    const RTB::Ellipsoid<float> ellipsoid(1.0F, 1.0F, 1.0F);
    const RTB::Ray<float, 3> ray({3.0F, 0.0F, 0.0F}, {-1.0F, 0.0F, 0.0F});
    const auto result = ellipsoid.intersectRay(ray);
    ASSERT_TRUE(result.has_value());

    if (result.has_value()) {
        const auto& hits = *result;
        float t1 = hits.at(0);
        float t2 = hits.at(1);
        if (t1 > t2) {
            std::swap(t1, t2);
        }
        EXPECT_NEAR(t1, 2.0F, 1e-5F);
        EXPECT_NEAR(t2, 4.0F, 1e-5F);
    }
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
    RTB::EllipseParams<double> ep = sphere.intersectPlane(xy);

    EXPECT_NEAR(ep.semi_axis_lengths[0], 1.0, k_med);
    EXPECT_NEAR(ep.semi_axis_lengths[1], 1.0, k_med);

    // Center should be at origin for z=0 plane
    EXPECT_NEAR(ep.center[0], 0.0, k_med);
    EXPECT_NEAR(ep.center[1], 0.0, k_med);
    EXPECT_NEAR(ep.center[2], 0.0, k_med);
}

// XY plane through prolate 2:1:1 → ellipse with semi-axes 2 and 1
TEST_F(EllipsoidPlaneTest, ProlateXYPlaneSemiAxes) {
    RTB::Plane<double> xy(0.0, 0.0, 1.0, 0.0);
    xy.normalize();
    RTB::EllipseParams<double> ep = prolate.intersectPlane(xy);

    const double sa0 = ep.semi_axis_lengths[0];
    const double sa1 = ep.semi_axis_lengths[1];
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
    RTB::EllipseParams<double> ep = prolate.intersectPlane(yz);

    EXPECT_NEAR(ep.semi_axis_lengths[0], 1.0, k_med);
    EXPECT_NEAR(ep.semi_axis_lengths[1], 1.0, k_med);
}

// Normal vector of returned ellipse matches plane normal
TEST_F(EllipsoidPlaneTest, NormalVectorMatchesPlane) {
    RTB::Plane<double> xy(0.0, 0.0, 1.0, 0.0);
    xy.normalize();
    RTB::EllipseParams<double> ep = sphere.intersectPlane(xy);

    // Normal should be (0, 0, ±1)
    EXPECT_NEAR(std::abs(ep.normal[2]), 1.0, k_tight);
    EXPECT_NEAR(ep.normal[0], 0.0, k_tight);
    EXPECT_NEAR(ep.normal[1], 0.0, k_tight);
}

// Semi-axis vectors are orthogonal to each other and to the normal
TEST_F(EllipsoidPlaneTest, SemiAxesOrthogonality) {
    RTB::Plane<double> xy(0.0, 0.0, 1.0, 0.0);
    xy.normalize();
    RTB::EllipseParams<double> ep = prolate.intersectPlane(xy);

    const double dot_axes = RTB::dotProduct(ep.semi_axes[0], ep.semi_axes[1]);
    EXPECT_NEAR(dot_axes, 0.0, k_med);

    const double dot_ax0_n = RTB::dotProduct(ep.semi_axes[0], ep.normal);
    const double dot_ax1_n = RTB::dotProduct(ep.semi_axes[1], ep.normal);
    EXPECT_NEAR(dot_ax0_n, 0.0, k_med);
    EXPECT_NEAR(dot_ax1_n, 0.0, k_med);
}

// Off-center plane: x=0 offset. Center should shift accordingly.
TEST_F(EllipsoidPlaneTest, OffCenterPlane) {
    // Plane x = 0.5  →  1*x + 0*y + 0*z - 0.5 = 0  ⟹  (1,0,0,-0.5)
    // but Plane stores ax+by+cz+dimensions=0, so dimensions = -0.5
    // x=0 offset means the center of the cross-section sits at (0.5, 0, 0)
    RTB::Plane<double> px(1.0, 0.0, 0.0, -0.5);
    px.normalize();
    RTB::EllipseParams<double> ep = prolate.intersectPlane(px);

    EXPECT_NEAR(ep.center[0], 0.5, k_med);
    EXPECT_NEAR(ep.center[1], 0.0, k_med);
    EXPECT_NEAR(ep.center[2], 0.0, k_med);
}

// ============================================================
// ArcLength accuracy — GaussianQuadrature vs Ramanujan vs Polynomial
// ============================================================

// For a sphere, arc length between two surface points on a great circle
// equals radius * angular_separation.
// We test this by comparing GaussianQuadrature (reference) against known
// analytical values, then comparing methods against each other.

namespace {
class ArcLengthMethodTest : public ::testing::Test {
   protected:
    // All three flavours of the same sphere

    // For a sphere of radius 1, arc length of a quarter great-circle = pi/2
    // Points on the sphere: (1,0,0) and (0,1,0), both on the XY equator.
    // The diffraction plane through origin, p1, and p2 is the XY plane.
    void SetUp() override {
        xy_plane.normalize();
    }

   private:
    RTB::Ellipsoid<double, RTB::ArcLengthMethod::GaussianQuadrature> gq{
        1.0, 1.0, 1.0};
    RTB::Ellipsoid<double, RTB::ArcLengthMethod::Ramanujan> ram{1.0, 1.0, 1.0};
    RTB::Ellipsoid<double, RTB::ArcLengthMethod::Polynomial> poly{
        1.0, 1.0, 1.0};
    RTB::Point<double, 3> p1{1.0, 0.0, 0.0};
    RTB::Point<double, 3> p2{0.0, 1.0, 0.0};
    RTB::Plane<double> xy_plane{0.0, 0.0, 1.0, 0.0};
};
}  // namespace

// GaussianQuadrature should be very close to the analytical pi/2 for a sphere
// The arc length is private, so we exercise it indirectly via tracePath.
// Instead we test ArcLength indirectly through tracePath path lengths vs
// known geometry (see tracePath tests below).

// Direct comparison: sphere full perimeter via each method
// Perimeter of a circle radius 1 = 2*pi.
// We verify by summing four quarter-arc lengths.
// NOTE: ArcLength is private — we access it indirectly via tracePath.
// See the dedicated tracePath tests for numerical validation.

// ============================================================
// GaussianQuadrature: known arc lengths on a sphere via tracePath
// ============================================================

// For a sphere of radius R, a quarter great-circle arc has length pi*R/2.
// Source directly above the north pole, ear on the equator.
// The path is unoccluded when the ear is on the near hemisphere, occluded
// otherwise. We arrange geometry to force occlusion.

TEST(EllipsoidArcLength, SphereQuarterArcGaussianQuadrature) {
    // Sphere radius 1. Source at (2,0,0). Ear at (-1,0,0) (far side —
    // occluded). The diffraction path wraps ±90° around the equator to reach
    // (-1,0,0), so arclength should be pi/2 ≈ 1.5708 for both paths
    // (symmetrical in XZ plane, or XY plane depending on tracePath internals).
    // We just check that arclength > 0 and pathlength is self-consistent.
    const RTB::Ellipsoid<double, RTB::ArcLengthMethod::GaussianQuadrature>
        sphere(1.0, 1.0, 1.0);

    const RTB::Point<double, 3> source{2.0, 0.0, 0.0};
    const RTB::Point<double, 3> left_ear{
        -1.0, 0.0, 0.0};  // antipodal — occluded
    const std::array<RTB::Point<double, 3>, 2> ears = {left_ear, left_ear};

    auto res = sphere.tracePath(source, ears);

    for (const auto& path : res.left_ear_paths) {
        EXPECT_GT(path.arclength, 0.0)
            << "Arclength should be positive (occluded path)";
        EXPECT_GE(path.pathlength, path.arclength) << "pathlength >= arclength";
    }

    // Both paths around antipodal point are symmetric: arc = pi/2 each
    const double expected_arc = RTB::pi / 2.0;
    EXPECT_NEAR(res.left_ear_paths[0].arclength, expected_arc, 1e-4);
    EXPECT_NEAR(res.left_ear_paths[1].arclength, expected_arc, 1e-4);
}

// For a sphere, arc length of half a great circle = pi.
TEST(EllipsoidArcLength, SphereHalfArcGaussianQuadrature) {
    // Source at (0, 2, 0) — above Y axis.
    // Ear at (0, -1, 0) — antipodal on Y axis. Occluded.
    // The shortest arc around the sphere from tangent point to ear = pi/2,
    // and the total arc (tangent_point is at (0,1,0)) to ear at (0,-1,0) = pi.
    const RTB::Ellipsoid<double, RTB::ArcLengthMethod::GaussianQuadrature>
        sphere(1.0, 1.0, 1.0);

    const RTB::Point<double, 3> source{0.0, 2.0, 0.0};
    const RTB::Point<double, 3> ear{0.0, -1.0, 0.0};
    const std::array<RTB::Point<double, 3>, 2> ears = {ear, ear};

    auto res = sphere.tracePath(source, ears);

    // Shorter path arc should be pi/2, longer arc should also be pi/2
    // (by symmetry the two tangent paths are equal for antipodal ear)
    for (const auto& path : res.left_ear_paths) {
        EXPECT_NEAR(path.arclength, RTB::pi / 2.0, 1e-4);
    }
}

// ============================================================
// ArcLength method comparison: GQ vs Ramanujan on a sphere
// ============================================================

// On a near-circular ellipse, Ramanujan should agree with GQ within 0.01%.
TEST(EllipsoidArcLength, RamanujanVsGQNearCircle) {
    const RTB::Ellipsoid<double, RTB::ArcLengthMethod::GaussianQuadrature> gq(
        1.0, 1.0, 1.0);
    const RTB::Ellipsoid<double, RTB::ArcLengthMethod::Ramanujan> ram(
        1.0, 1.0, 1.0);

    const RTB::Point<double, 3> source{2.0, 0.0, 0.0};
    const RTB::Point<double, 3> ear{-1.0, 0.0, 0.0};
    const std::array<RTB::Point<double, 3>, 2> ears = {ear, ear};

    auto res_gq = gq.tracePath(source, ears);
    auto res_ram = ram.tracePath(source, ears);

    for (size_t ii = 0; ii < 2; ++ii) {
        const double gq_arc = res_gq.left_ear_paths[ii].arclength;
        const double ram_arc = res_ram.left_ear_paths[ii].arclength;
        EXPECT_NEAR(ram_arc, gq_arc, gq_arc * 1e-4)
            << "Ramanujan and GQ disagree by more than 0.01% on sphere";
    }
}

// On a highly eccentric ellipsoid, Polynomial degrades but GQ stays accurate.
// We verify GQ gives reasonable results and Polynomial error is acceptable
// within its documented limits (near-circular).
TEST(EllipsoidArcLength, PolynomialDegradeHighEccentricity) {
    // Very eccentric: 3:1 ratio — Polynomial is documented to degrade here
    const RTB::Ellipsoid<double, RTB::ArcLengthMethod::GaussianQuadrature> gq(
        3.0, 1.0, 1.0);
    const RTB::Ellipsoid<double, RTB::ArcLengthMethod::Polynomial> poly(
        3.0, 1.0, 1.0);

    const RTB::Point<double, 3> source{4.0, 0.0, 0.0};
    const RTB::Point<double, 3> ear{-3.0, 0.0, 0.0};
    const std::array<RTB::Point<double, 3>, 2> ears = {ear, ear};

    auto res_gq = gq.tracePath(source, ears);
    auto res_poly = poly.tracePath(source, ears);

    const double gq_arc = res_gq.left_ear_paths[0].arclength;
    const double poly_arc = res_poly.left_ear_paths[0].arclength;

    // Both should be positive and have the right rough scale
    EXPECT_GT(gq_arc, 0.0);
    EXPECT_GT(poly_arc, 0.0);

    // Polynomial error can be large for high eccentricity — document it
    const double rel_error = std::abs(poly_arc - gq_arc) / gq_arc;
    // We don't assert a hard limit on Polynomial here since it is documented
    // to degrade, but we record the test for observability.
    GTEST_LOG_(INFO) << "Polynomial relative error vs GQ (3:1 ellipsoid): "
                     << rel_error * 100.0 << "%";
}

// ============================================================
// tracePath — unoccluded paths
// ============================================================

TEST(EllipsoidTracePath, UnoccludedPathHasZeroArcLength) {
    // Source and ear both on same hemisphere — no occlusion.
    const RTB::Ellipsoid<double> sphere(1.0, 1.0, 1.0);

    const RTB::Point<double, 3> source{2.0, 0.0, 0.0};
    // Ear on the near side of the sphere
    const RTB::Point<double, 3> near_ear{1.0, 0.0, 0.0};
    const std::array<RTB::Point<double, 3>, 2> ears = {near_ear, near_ear};

    auto res = sphere.tracePath(source, ears);

    EXPECT_NEAR(res.left_ear_paths[0].arclength, 0.0, k_med);
    EXPECT_NEAR(res.right_ear_paths[0].arclength, 0.0, k_med);
}

TEST(EllipsoidTracePath, UnoccludedPathlengthEqualsDirectDistance) {
    const RTB::Ellipsoid<double> sphere(1.0, 1.0, 1.0);

    const RTB::Point<double, 3> source{3.0, 0.0, 0.0};
    const RTB::Point<double, 3> near_ear{1.0, 0.0, 0.0};
    const std::array<RTB::Point<double, 3>, 2> ears = {near_ear, near_ear};

    auto res = sphere.tracePath(source, ears);

    // Direct distance from source to ear = 2.0
    const double expected = RTB::distance2Points(source, near_ear);
    EXPECT_NEAR(res.left_ear_paths[0].pathlength, expected, k_med);
}

// ============================================================
// tracePath — occluded paths
// ============================================================

TEST(EllipsoidTracePath, OccludedPathShorterThanLonger) {
    const RTB::Ellipsoid<double> sphere(1.0, 1.0, 1.0);
    const RTB::Point<double, 3> source{2.0, 0.0, 0.0};
    const RTB::Point<double, 3> ear{-1.0, 0.0, 0.0};
    const std::array<RTB::Point<double, 3>, 2> ears = {ear, ear};

    auto res = sphere.tracePath(source, ears);

    EXPECT_LE(res.left_ear_paths[0].pathlength,
              res.left_ear_paths[1].pathlength)
        << "Primary path (index 0) should be <= secondary path (index 1)";
}

TEST(EllipsoidTracePath, TangentPointLiesOnSurface) {
    const RTB::Ellipsoid<double> sphere(1.0, 1.0, 1.0);
    const RTB::Point<double, 3> source{2.0, 0.0, 0.0};
    const RTB::Point<double, 3> ear{-1.0, 0.0, 0.0};
    const std::array<RTB::Point<double, 3>, 2> ears = {ear, ear};

    auto res = sphere.tracePath(source, ears);

    for (const auto& path : res.left_ear_paths) {
        EXPECT_TRUE(onSurface(path.tangent_point, 1.0, 1.0, 1.0))
            << "Tangent point is not on the ellipsoid surface";
    }
}

TEST(EllipsoidTracePath, PathlengthIsDirectPlusArc) {
    const RTB::Ellipsoid<double> sphere(1.0, 1.0, 1.0);
    const RTB::Point<double, 3> source{2.0, 0.0, 0.0};
    const RTB::Point<double, 3> ear{-1.0, 0.0, 0.0};
    const std::array<RTB::Point<double, 3>, 2> ears = {ear, ear};

    auto res = sphere.tracePath(source, ears);

    for (const auto& path : res.left_ear_paths) {
        const double direct_len = path.direct_ray.magnitude();
        EXPECT_NEAR(path.pathlength, direct_len + path.arclength, k_med);
    }
}

// Both ears independent — left and right results should be consistent
TEST(EllipsoidTracePath, LeftAndRightEarsAreIndependent) {
    const RTB::Ellipsoid<double> sphere(1.0, 1.0, 1.0);
    const RTB::Point<double, 3> source{2.0, 0.0, 0.0};

    // Left ear occluded, right ear unoccluded
    const RTB::Point<double, 3> left_ear{-1.0, 0.0, 0.0};
    const RTB::Point<double, 3> right_ear{1.0, 0.0, 0.0};
    const std::array<RTB::Point<double, 3>, 2> ears = {left_ear, right_ear};

    auto res = sphere.tracePath(source, ears);

    // Left ear — occluded: arclength > 0
    EXPECT_GT(res.left_ear_paths[0].arclength, 0.0);

    // Right ear — direct: arclength = 0
    EXPECT_NEAR(res.right_ear_paths[0].arclength, 0.0, k_med);
}

// ============================================================
// tracePath — symmetry checks
// ============================================================

// For a sphere, rotating source and ears by 90° about Z should give
// identical path lengths (rotational symmetry).
TEST(EllipsoidTracePath, RotationalSymmetryOnSphere) {
    const RTB::Ellipsoid<double> sphere(1.0, 1.0, 1.0);

    // Config A: source along +X
    {
        const RTB::Point<double, 3> src{2.0, 0.0, 0.0};
        const RTB::Point<double, 3> ear{-1.0, 0.0, 0.0};
        const std::array<RTB::Point<double, 3>, 2> ears = {ear, ear};
        auto res = sphere.tracePath(src, ears);
        const double pl_a = res.left_ear_paths[0].pathlength;

        // Config B: source along +Y (90° rotation)
        const RTB::Point<double, 3> src2{0.0, 2.0, 0.0};
        const RTB::Point<double, 3> ear2{0.0, -1.0, 0.0};
        const std::array<RTB::Point<double, 3>, 2> ears2 = {ear2, ear2};
        auto res2 = sphere.tracePath(src2, ears2);
        const double pl_b = res2.left_ear_paths[0].pathlength;

        EXPECT_NEAR(pl_a, pl_b, 1e-6)
            << "Sphere path lengths should be equal under 90° rotation";
    }
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
    const std::array<RTB::Point<double, 3>, 2> ears_s = {ear_s, ear_s};

    const RTB::Point<double, 3> src_l{2.0 * scale, 0.0, 0.0};
    const RTB::Point<double, 3> ear_l{-1.0 * scale, 0.0, 0.0};
    const std::array<RTB::Point<double, 3>, 2> ears_l = {ear_l, ear_l};

    auto res_s = sphere_small.tracePath(src_s, ears_s);
    auto res_l = sphere_large.tracePath(src_l, ears_l);

    const double pl_s = res_s.left_ear_paths[0].pathlength;
    const double pl_l = res_l.left_ear_paths[0].pathlength;

    EXPECT_NEAR(pl_l / pl_s, scale, 1e-4)
        << "Path length should scale linearly with ellipsoid size";
}

// ============================================================
// Ellipsoid type aliases
// ============================================================

TEST(EllipsoidAliasTest, EllipsoidfCompiles) {
    const RTB::Ellipsoidf ellipsoid(1.0F, 2.0F, 3.0F);
    const auto& dimensions = ellipsoid.getDimensions();
    EXPECT_FLOAT_EQ(dimensions[0], 1.0F);
    EXPECT_FLOAT_EQ(dimensions[1], 2.0F);
    EXPECT_FLOAT_EQ(dimensions[2], 3.0F);
}

TEST(EllipsoidAliasTest, EllipsoiddRamanujanCompiles) {
    const RTB::EllipsoiddRamanujan ellipsoid(1.0, 1.0, 1.0);
    const RTB::Ray<double, 3> ray({3.0, 0.0, 0.0}, {-1.0, 0.0, 0.0});
    EXPECT_TRUE(ellipsoid.intersectRay(ray).has_value());
}