// GOOGLETEST
#include <gtest/gtest.h>

// LOCAL
// Smoke test: confirms the umbrella header compiles as a single
// translation unit and that a type from each header it should
// transitively or directly declare is actually usable.
#include <RTB/RTB.hpp>

TEST(RTBUmbrellaHeaderTest, PullsInVector) {
    const RTB::Vector<double, 3> vector{1.0, 2.0, 3.0};
    EXPECT_EQ(vector[0], 1.0);
}

TEST(RTBUmbrellaHeaderTest, PullsInPoint) {
    const RTB::Point<double, 3> point{1.0, 2.0, 3.0};
    EXPECT_EQ(point[0], 1.0);
}

TEST(RTBUmbrellaHeaderTest, PullsInColour) {
    const RTB::Colour8_3 colour{10, 20, 30};
    EXPECT_EQ(colour.getComponents()[0], 10);
}

TEST(RTBUmbrellaHeaderTest, PullsInEllipsoid) {
    const RTB::Ellipsoid<double> ellipsoid(1.0, 1.0, 1.0);
    EXPECT_EQ(ellipsoid.getDimensions()[0], 1.0);
}

TEST(RTBUmbrellaHeaderTest, PullsInMixer) {
    RTB::MatrixMixer<float, 2, 2> mixer;
    mixer.prepare(44100.0F, 10.0F);
    EXPECT_FALSE(mixer.isRamping());
}

TEST(RTBUmbrellaHeaderTest, PullsInStandards) {
    EXPECT_EQ(RTB::speed_of_sound, 344.0);
}

TEST(RTBUmbrellaHeaderTest, PullsInTimer) {
    RTB::Timer timer;
    EXPECT_GE(timer.elapsed(), 0.0);
}
