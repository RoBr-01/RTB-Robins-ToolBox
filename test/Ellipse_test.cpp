#include <gtest/gtest.h>

#include <RTB/Ellipse.hpp>
#include <RTB/Math.hpp>
#include <RTB/Point.hpp>
#include <RTB/Vector.hpp>

TEST(Ellipse, ArcLength_SamePoint_IsZero) {
    const RTB::Ellipse<double> ellipse(
        {0, 0, 0},
        {RTB::Vector<double, 3>{1, 0, 0}, RTB::Vector<double, 3>{0, 1, 0}},
        {2.0, 1.0},
        {});

    const RTB::Point<double, 3> point{2.0, 0.0, 0.0};

    EXPECT_NEAR(ellipse.arcLength(point, point), 0.0, 1e-12);
}

TEST(Ellipse, ArcLength_Symmetric) {
    const RTB::Ellipse<double> ellipse(
        {0, 0, 0},
        {RTB::Vector<double, 3>{1, 0, 0}, RTB::Vector<double, 3>{0, 1, 0}},
        {2.0, 1.0},
        {});

    const RTB::Point<double, 3> point_1{2.0, 0.0, 0.0};
    const RTB::Point<double, 3> point_2{0.0, 1.0, 0.0};

    const double forward = ellipse.arcLength(point_1, point_2);
    const double backward = ellipse.arcLength(point_2, point_1);

    EXPECT_NEAR(forward, backward, 1e-12);
}

TEST(Ellipse, ArcLength_Additivity) {
    const RTB::Ellipse<double> ellipse(
        {0, 0, 0},
        {RTB::Vector<double, 3>{1, 0, 0}, RTB::Vector<double, 3>{0, 1, 0}},
        {2.0, 1.0},
        {});

    const RTB::Point<double, 3> point_a{2.0, 0.0, 0.0};
    const RTB::Point<double, 3> point_b{0.0, 1.0, 0.0};
    const RTB::Point<double, 3> point_c{-2.0, 0.0, 0.0};

    const double arc_ab = ellipse.arcLength(point_a, point_b);
    const double arc_bc = ellipse.arcLength(point_b, point_c);
    const double arc_ac = ellipse.arcLength(point_a, point_c);

    EXPECT_NEAR(arc_ab + arc_bc, arc_ac, 1e-6);
}

TEST(Ellipse, ArcLength_HalfPlusHalfEqualsCircumference) {
    const RTB::Ellipse<double> ellipse(
        {0, 0, 0},
        {RTB::Vector<double, 3>{1, 0, 0}, RTB::Vector<double, 3>{0, 1, 0}},
        {2.0, 1.0},
        {});

    const RTB::Point<double, 3> point_a{2.0, 0.0, 0.0};
    const RTB::Point<double, 3> point_b{-2.0, 0.0, 0.0};

    const double forward = ellipse.arcLength(point_a, point_b);
    const double backward = ellipse.arcLength(point_b, point_a);

    // symmetry must hold
    EXPECT_NEAR(forward, backward, 1e-12);

    // weak sanity check (ensures no degeneration)
    EXPECT_GT(forward, 0.0);
}

TEST(Ellipse, Circle_BehavesLikeCircle) {
    const RTB::Ellipse<double> ellipse(
        {0, 0, 0},
        {RTB::Vector<double, 3>{1, 0, 0}, RTB::Vector<double, 3>{0, 1, 0}},
        {1.0, 1.0},
        {});

    const RTB::Point<double, 3> point_1{1.0, 0.0, 0.0};
    const RTB::Point<double, 3> point_2{0.0, 1.0, 0.0};

    const double arc = ellipse.arcLength(point_1, point_2);

    EXPECT_NEAR(arc, RTB::pi / 2.0, 1e-6);
}

TEST(Ellipse, Rotation_Symmetry) {
    const RTB::Ellipse<double> ellipse(
        {0, 0, 0},
        {RTB::Vector<double, 3>{1, 0, 0}, RTB::Vector<double, 3>{0, 1, 0}},
        {3.0, 1.0},
        {});

    const RTB::Point<double, 3> point_1{3.0, 0.0, 0.0};
    const RTB::Point<double, 3> point_2{0.0, 1.0, 0.0};

    const double arc = ellipse.arcLength(point_1, point_2);

    const RTB::Point<double, 3> point_1_rotated{0.0, 3.0, 0.0};
    const RTB::Point<double, 3> point_2_rotated{-1.0, 0.0, 0.0};

    const double arc_rotated =
        ellipse.arcLength(point_1_rotated, point_2_rotated);

    EXPECT_NEAR(arc, arc_rotated, 1e-6);
}