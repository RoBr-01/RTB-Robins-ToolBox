#include <gtest/gtest.h>

#include <RTB/Timer.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <regex>
#include <sstream>
#include <streambuf>
#include <string>
#include <thread>

namespace {

constexpr double minsleepslack = 0.80;  // measured >= 80% of requested sleep
constexpr double maxsleepslack = 10.0;  // generous upper bound

// Relative comparison helper.
bool near(double time_a,
          double time_b,
          double abs_tol = 1e-9,
          double rel_tol = 1e-6) {
    const double diff = std::abs(time_a - time_b);
    if (diff <= abs_tol) {
        return true;
    }
    return diff <= rel_tol * std::max(std::abs(time_a), std::abs(time_b));
}

struct ParsedTimerOutput {
    std::string label;
    double value{};
    std::string unit;
};

// Parses:
// [TIMER] MyLabel - 12.345 ms
ParsedTimerOutput parseTimerOutputs(const std::string& output) {
    static const std::regex re(
        R"(^\[TIMER\]\s*(.*?)\s*-\s*([-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?)\s+(s|ms|us|ns)\s*$)");

    std::smatch match;
    EXPECT_TRUE(std::regex_search(output, match, re))
        << "Could not parse timer output: '" << output << "'";

    ParsedTimerOutput result;
    result.label = match[1].str();
    result.value = std::stod(match[2].str());
    result.unit = match[3].str();
    return result;
}

}  // namespace

namespace {
class TimerTest : public ::testing::Test {
   protected:
    void SetUp() override {
        old_buf_ = std::cout.rdbuf(capture_.rdbuf());
    }

    void TearDown() override {
        std::cout.rdbuf(old_buf_);
    }

    std::string output() const {
        return capture_.str();
    }

    void clearOutput() {
        capture_.str("");
        capture_.clear();
    }

    static void sleepFor(std::chrono::milliseconds duration) {
        std::this_thread::sleep_for(duration);
    }

    static void expectElapsedWithin(double measured,
                                    double expected,
                                    double min_factor = minsleepslack,
                                    double max_factor = maxsleepslack) {
        EXPECT_GE(measured, expected * min_factor);
        EXPECT_LE(measured, expected * max_factor);
    }

   private:
    std::stringstream capture_;
    std::streambuf* old_buf_{nullptr};
};

}  // namespace

// -----------------------------------------------------------------------------
// RTB::detail::durationLabel
// -----------------------------------------------------------------------------

TEST(TimerDetail, DurationLabelMappings) {
    EXPECT_STREQ(RTB::detail::durationLabel<std::chrono::seconds>(), "s");
    EXPECT_STREQ(RTB::detail::durationLabel<std::chrono::milliseconds>(), "ms");
    EXPECT_STREQ(RTB::detail::durationLabel<std::chrono::microseconds>(), "us");
    EXPECT_STREQ(RTB::detail::durationLabel<std::chrono::nanoseconds>(), "ns");

    // Unmapped durations default to "ns".
    EXPECT_STREQ(RTB::detail::durationLabel<std::chrono::minutes>(), "ns");
}

// -----------------------------------------------------------------------------
// RTB::Timer core behavior
// -----------------------------------------------------------------------------

TEST_F(TimerTest, StartsAutomatically) {
    const RTB::Timer timer;

    sleepFor(std::chrono::milliseconds(20));

    const double elapsed_ms = timer.elapsed<std::chrono::milliseconds>();

    expectElapsedWithin(elapsed_ms, 20.0);
}

TEST_F(TimerTest, StartResetsElapsedTime) {
    RTB::Timer timer;

    sleepFor(std::chrono::milliseconds(20));
    const double before_reset = timer.elapsed<std::chrono::milliseconds>();

    timer.start();

    sleepFor(std::chrono::milliseconds(10));
    const double after_reset = timer.elapsed<std::chrono::milliseconds>();

    EXPECT_GT(before_reset, after_reset);
    expectElapsedWithin(after_reset, 10.0);
}

TEST_F(TimerTest, ElapsedIsMonotonic) {
    const RTB::Timer timer;

    const double t0 = timer.elapsed<std::chrono::nanoseconds>();

    sleepFor(std::chrono::milliseconds(5));
    const double t1 = timer.elapsed<std::chrono::nanoseconds>();

    sleepFor(std::chrono::milliseconds(5));
    const double t2 = timer.elapsed<std::chrono::nanoseconds>();

    EXPECT_GE(t1, t0);
    EXPECT_GE(t2, t1);
}

TEST_F(TimerTest, ImmediateElapsedIsNonNegative) {
    const RTB::Timer timer;

    const double elapsed_ns = timer.elapsed<std::chrono::nanoseconds>();

    EXPECT_GE(elapsed_ns, 0.0);
    EXPECT_TRUE(std::isfinite(elapsed_ns));
}

// -----------------------------------------------------------------------------
// Unit scaling
// -----------------------------------------------------------------------------

TEST_F(TimerTest, UnitScalingIsConsistent) {
    const RTB::Timer timer;

    sleepFor(std::chrono::milliseconds(50));

    const double seconds = timer.elapsed<std::chrono::seconds>();
    const double milliseconds = timer.elapsed<std::chrono::milliseconds>();
    const double microseconds = timer.elapsed<std::chrono::microseconds>();
    const double nanoseconds = timer.elapsed<std::chrono::nanoseconds>();

    EXPECT_TRUE(near(seconds * 1e3, milliseconds, 5.0, 0.05));
    EXPECT_TRUE(near(milliseconds * 1e3, microseconds, 5000.0, 0.05));
    EXPECT_TRUE(near(microseconds * 1e3, nanoseconds, 5e6, 0.05));
}

TEST_F(TimerTest, DefaultElapsedUsesNanoseconds) {
    const RTB::Timer timer;

    sleepFor(std::chrono::milliseconds(5));

    const double default_elapsed = timer.elapsed<>();
    const double explicit_elapsed = timer.elapsed<std::chrono::nanoseconds>();

    // Two calls occur at slightly different times, so explicit should be >=.
    EXPECT_GE(explicit_elapsed, default_elapsed);
}

// -----------------------------------------------------------------------------
// RTB::Timer::print
// -----------------------------------------------------------------------------

TEST_F(TimerTest, PrintWithoutLabel) {
    const RTB::Timer timer;

    sleepFor(std::chrono::milliseconds(5));
    timer.print<std::chrono::milliseconds>();

    const auto parsed = parseTimerOutputs(output());

    EXPECT_EQ(parsed.label, "");
    EXPECT_EQ(parsed.unit, "ms");
    EXPECT_GE(parsed.value, 0.0);
}

TEST_F(TimerTest, PrintWithLabel) {
    const RTB::Timer timer;

    sleepFor(std::chrono::milliseconds(5));
    timer.print<std::chrono::microseconds>("MyOperation");

    const auto parsed = parseTimerOutputs(output());

    EXPECT_EQ(parsed.label, "MyOperation");
    EXPECT_EQ(parsed.unit, "us");
    EXPECT_GT(parsed.value, 0.0);
}

// -----------------------------------------------------------------------------
// RTB::ScopedTimer
// -----------------------------------------------------------------------------

TEST_F(TimerTest, ScopedTimerPrintsOnDestruction) {
    {
        const RTB::ScopedTimer<std::chrono::milliseconds> timer("ScopedOp");
        sleepFor(std::chrono::milliseconds(10));
    }

    const auto parsed = parseTimerOutputs(output());

    EXPECT_EQ(parsed.label, "ScopedOp");
    EXPECT_EQ(parsed.unit, "ms");
    EXPECT_GT(parsed.value, 0.0);
}

TEST_F(TimerTest, NestedScopedTimersPrintInReverseDestructionOrder) {
    {
        const RTB::ScopedTimer<std::chrono::milliseconds> outer("Outer");
        sleepFor(std::chrono::milliseconds(2));

        {
            const RTB::ScopedTimer<std::chrono::microseconds> inner("Inner");
            sleepFor(std::chrono::milliseconds(2));
        }

        sleepFor(std::chrono::milliseconds(2));
    }

    const std::string out = output();

    const auto inner_pos = out.find("Inner");
    const auto outer_pos = out.find("Outer");

    ASSERT_NE(inner_pos, std::string::npos);
    ASSERT_NE(outer_pos, std::string::npos);

    // Inner timer destructs first, so its output appears first.
    EXPECT_LT(inner_pos, outer_pos);
}

// -----------------------------------------------------------------------------
// Numerical sanity of RTB::ScopedTimer output
// -----------------------------------------------------------------------------

TEST_F(TimerTest, ScopedTimerMeasuresExpectedDuration) {
    {
        const RTB::ScopedTimer<std::chrono::milliseconds> timer("Measured");
        sleepFor(std::chrono::milliseconds(25));
    }

    const auto parsed = parseTimerOutputs(output());

    EXPECT_EQ(parsed.unit, "ms");
    expectElapsedWithin(parsed.value, 25.0);
}

// -----------------------------------------------------------------------------
// Stress / repeated use
// -----------------------------------------------------------------------------

TEST_F(TimerTest, RepeatedStartAndElapsed) {
    RTB::Timer timer;

    for (int i = 0; i < 20; ++i) {
        timer.start();
        sleepFor(std::chrono::milliseconds(2));

        const double elapsed = timer.elapsed<std::chrono::milliseconds>();

        expectElapsedWithin(elapsed, 2.0);
    }
}