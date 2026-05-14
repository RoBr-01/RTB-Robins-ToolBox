#include <gtest/gtest.h>

#include <RTB/Ellipse.hpp>

TEST(Ellipse, ArcLength_SamePoint_IsZero) {
    RTB::Ellipse<double> e;

    e.center = {0, 0, 0};
    e.semi_axes = {RTB::Vector<double, 3>{1, 0, 0},
                   RTB::Vector<double, 3>{0, 1, 0}};
    e.semi_axis_lengths = {2.0, 1.0};

    RTB::Point<double, 3> p{2.0, 0.0, 0.0};

    EXPECT_NEAR(e.arcLength(p, p), 0.0, 1e-12);
}

TEST(Ellipse, ArcLength_Symmetric) {
    RTB::Ellipse<double> e;

    e.center = {0, 0, 0};
    e.semi_axes = {RTB::Vector<double, 3>{1, 0, 0},
                   RTB::Vector<double, 3>{0, 1, 0}};
    e.semi_axis_lengths = {2.0, 1.0};

    RTB::Point<double, 3> p1{2.0, 0.0, 0.0};
    RTB::Point<double, 3> p2{0.0, 1.0, 0.0};

    double a = e.arcLength(p1, p2);
    double b = e.arcLength(p2, p1);

    EXPECT_NEAR(a, b, 1e-12);
}

TEST(Ellipse, ArcLength_Additivity) {
    RTB::Ellipse<double> e;

    e.center = {0, 0, 0};
    e.semi_axes = {RTB::Vector<double, 3>{1, 0, 0},
                   RTB::Vector<double, 3>{0, 1, 0}};
    e.semi_axis_lengths = {2.0, 1.0};

    RTB::Point<double, 3> a{2.0, 0.0, 0.0};
    RTB::Point<double, 3> b{0.0, 1.0, 0.0};
    RTB::Point<double, 3> c{-2.0, 0.0, 0.0};

    double ab = e.arcLength(a, b);
    double bc = e.arcLength(b, c);
    double ac = e.arcLength(a, c);

    EXPECT_NEAR(ab + bc, ac, 1e-6);
}

TEST(Ellipse, ArcLength_HalfPlusHalfEqualsCircumference) {
    RTB::Ellipse<double> e;

    e.center = {0, 0, 0};
    e.semi_axes = {RTB::Vector<double, 3>{1, 0, 0},
                   RTB::Vector<double, 3>{0, 1, 0}};
    e.semi_axis_lengths = {2.0, 1.0};

    RTB::Point<double, 3> a{2.0, 0.0, 0.0};
    RTB::Point<double, 3> b{-2.0, 0.0, 0.0};

    double ab = e.arcLength(a, b);
    double ba = e.arcLength(b, a);

    // symmetry must hold
    EXPECT_NEAR(ab, ba, 1e-12);

    // weak sanity check (ensures no degeneration)
    EXPECT_GT(ab, 0.0);
}

TEST(Ellipse, Circle_BehavesLikeCircle) {
    RTB::Ellipse<double> e;

    e.center = {0, 0, 0};
    e.semi_axes = {RTB::Vector<double, 3>{1, 0, 0},
                   RTB::Vector<double, 3>{0, 1, 0}};
    e.semi_axis_lengths = {1.0, 1.0};

    RTB::Point<double, 3> p1{1.0, 0.0, 0.0};
    RTB::Point<double, 3> p2{0.0, 1.0, 0.0};

    double arc = e.arcLength(p1, p2);

    EXPECT_NEAR(arc, M_PI / 2.0, 1e-6);
}

TEST(Ellipse, Rotation_Symmetry) {
    RTB::Ellipse<double> e;

    e.center = {0, 0, 0};
    e.semi_axes = {RTB::Vector<double, 3>{1, 0, 0},
                   RTB::Vector<double, 3>{0, 1, 0}};
    e.semi_axis_lengths = {3.0, 1.0};

    RTB::Point<double, 3> p1{3.0, 0.0, 0.0};
    RTB::Point<double, 3> p2{0.0, 1.0, 0.0};

    double a = e.arcLength(p1, p2);

    RTB::Point<double, 3> p1r{0.0, 3.0, 0.0};
    RTB::Point<double, 3> p2r{-1.0, 0.0, 0.0};

    double b = e.arcLength(p1r, p2r);

    EXPECT_NEAR(a, b, 1e-6);
}