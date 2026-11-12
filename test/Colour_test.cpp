// GOOGLETEST
#include <gtest/gtest.h>

// STL
#include <cstdint>

// LOCAL
#include <RTB/Colour.hpp>

using Colour4 = RTB::Colour<double, 4>;

TEST(ColourTest, DefaultConstructorIsZero) {
    const Colour4 colour;
    for (const auto& comp : colour.getComponents()) {
        EXPECT_EQ(comp, 0.0);
    }
}

TEST(ColourTest, InitializerListConstructor) {
    const Colour4 colour{1.0, 2.0, 3.0, 4.0};
    const auto& components = colour.getComponents();

    EXPECT_EQ(components[0], 1.0);
    EXPECT_EQ(components[1], 2.0);
    EXPECT_EQ(components[2], 3.0);
    EXPECT_EQ(components[3], 4.0);
}

TEST(ColourTest, ShortInitializerListLeavesRemainderZero) {
    const Colour4 colour{1.0, 2.0};
    const auto& components = colour.getComponents();

    EXPECT_EQ(components[0], 1.0);
    EXPECT_EQ(components[1], 2.0);
    EXPECT_EQ(components[2], 0.0);
    EXPECT_EQ(components[3], 0.0);
}

TEST(ColourTest, LongInitializerListIgnoresExtraValues) {
    const Colour4 colour{1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
    const auto& components = colour.getComponents();

    EXPECT_EQ(components[0], 1.0);
    EXPECT_EQ(components[1], 2.0);
    EXPECT_EQ(components[2], 3.0);
    EXPECT_EQ(components[3], 4.0);
}

TEST(ColourTest, ClampLowerBound) {
    Colour4 colour{-5.0, 0.5, 300.0, 2.0};
    colour.clamp(0.0, 255.0);
    const auto& components = colour.getComponents();

    EXPECT_EQ(components[0], 0.0);
    EXPECT_EQ(components[1], 0.5);
    EXPECT_EQ(components[2], 255.0);
    EXPECT_EQ(components[3], 2.0);
}

TEST(ColourTest, Colour8AliasStoresUnsignedByteRange) {
    const RTB::Colour8_4 colour{10, 20, 30, 255};
    const auto& components = colour.getComponents();

    EXPECT_EQ(components[0], 10);
    EXPECT_EQ(components[1], 20);
    EXPECT_EQ(components[2], 30);
    EXPECT_EQ(components[3], 255);
}
