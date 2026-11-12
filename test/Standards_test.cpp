// GOOGLETEST
#include <gtest/gtest.h>

// LOCAL
#include <RTB/Standards.hpp>

TEST(StandardsTest, PhysicalConstants) {
    EXPECT_DOUBLE_EQ(RTB::speed_of_sound, 344.0);
    EXPECT_DOUBLE_EQ(RTB::speed_of_light, 299'792'458.0);
}

TEST(StandardsTest, AxesOrder) {
    EXPECT_EQ(static_cast<int>(RTB::Axes::X), 0);
    EXPECT_EQ(static_cast<int>(RTB::Axes::Y), 1);
    EXPECT_EQ(static_cast<int>(RTB::Axes::Z), 2);
}

TEST(StandardsTest, DirectionsOrder) {
    EXPECT_EQ(static_cast<int>(RTB::Directions::Left), 0);
    EXPECT_EQ(static_cast<int>(RTB::Directions::Right), 1);
    EXPECT_EQ(static_cast<int>(RTB::Directions::Front), 2);
    EXPECT_EQ(static_cast<int>(RTB::Directions::Back), 3);
    EXPECT_EQ(static_cast<int>(RTB::Directions::Up), 4);
    EXPECT_EQ(static_cast<int>(RTB::Directions::Down), 5);
}

// Vertices encodes a Left/Right, Front/Back, Down/Up bit pattern
// (LFD, RFD, LBD, RBD, LFU, RFU, LBU, RBU). Pinned here so a future
// reorder that breaks this convention fails loudly instead of silently
// corrupting any code that later indexes an 8-element array by it.
TEST(StandardsTest, VerticesOrder) {
    EXPECT_EQ(static_cast<int>(RTB::Vertices::LFD), 0);
    EXPECT_EQ(static_cast<int>(RTB::Vertices::RFD), 1);
    EXPECT_EQ(static_cast<int>(RTB::Vertices::LBD), 2);
    EXPECT_EQ(static_cast<int>(RTB::Vertices::RBD), 3);
    EXPECT_EQ(static_cast<int>(RTB::Vertices::LFU), 4);
    EXPECT_EQ(static_cast<int>(RTB::Vertices::RFU), 5);
    EXPECT_EQ(static_cast<int>(RTB::Vertices::LBU), 6);
    EXPECT_EQ(static_cast<int>(RTB::Vertices::RBU), 7);
}
