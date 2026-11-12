// GOOGLETEST
#include <gtest/gtest.h>

// STL
#include <array>
#include <cmath>

// LOCAL
#include <RTB/Mixer.hpp>

namespace {

constexpr double k_eps = 1e-9;

}  // namespace

TEST(MixerTest, DefaultGainIsZero) {
    RTB::MatrixMixer<double, 2, 2> mixer;
    mixer.prepare(44100.0, 10.0);

    EXPECT_NEAR(mixer.getGain(0, 0), 0.0, k_eps);
    EXPECT_NEAR(mixer.getGain(0, 1), 0.0, k_eps);
    EXPECT_NEAR(mixer.getGain(1, 0), 0.0, k_eps);
    EXPECT_NEAR(mixer.getGain(1, 1), 0.0, k_eps);
}

TEST(MixerTest, SetGainUpdatesTargetImmediately) {
    RTB::MatrixMixer<double, 2, 2> mixer;
    mixer.prepare(44100.0, 10.0);
    mixer.setGain(0, 0, 1.0);

    EXPECT_NEAR(mixer.getMatrix()[0], 1.0, k_eps);
}

TEST(MixerTest, GainRampsToTargetOverExactSampleCount) {
    RTB::MatrixMixer<double, 1, 1> mixer;
    // ramp_time_ms chosen so ramp_samples is exactly 10 at this rate.
    mixer.prepare(1000.0, 10.0);
    mixer.setGain(0, 0, 1.0);

    EXPECT_TRUE(mixer.isRamping());

    std::array<double, 1> in{1.0};
    std::array<double, 1> out{};

    for (int i = 0; i < 10; ++i) {
        mixer.processSample(in, out);
    }

    EXPECT_FALSE(mixer.isRamping());
    EXPECT_NEAR(mixer.getGain(0, 0), 1.0, k_eps);
    EXPECT_NEAR(out[0], 1.0, k_eps);
}

TEST(MixerTest, GainRampIsMonotonicTowardsTarget) {
    RTB::MatrixMixer<double, 1, 1> mixer;
    mixer.prepare(1000.0, 10.0);
    mixer.setGain(0, 0, 1.0);

    std::array<double, 1> in{1.0};
    std::array<double, 1> out{};

    double previous_gain = mixer.getGain(0, 0);
    for (int i = 0; i < 10; ++i) {
        mixer.processSample(in, out);
        const double current_gain = mixer.getGain(0, 0);
        EXPECT_GE(current_gain, previous_gain);
        previous_gain = current_gain;
    }
}

TEST(MixerTest, ProcessSampleAppliesIdentityMatrix) {
    RTB::MatrixMixer<double, 2, 2> mixer;
    mixer.prepare(44100.0, 0.0);  // zero ramp time -> at least 1 sample
    mixer.setMatrix({1.0, 0.0, 0.0, 1.0});

    std::array<double, 2> in{3.0, 4.0};
    std::array<double, 2> out{};

    // Drive the (minimum 1-sample) ramp to completion.
    mixer.processSample(in, out);
    mixer.processSample(in, out);

    EXPECT_NEAR(out[0], 3.0, k_eps);
    EXPECT_NEAR(out[1], 4.0, k_eps);
}

TEST(MixerTest, ProcessSampleMixesInputsAccordingToMatrix) {
    RTB::MatrixMixer<double, 2, 1> mixer;
    mixer.prepare(44100.0, 0.0);
    // Single output = sum of both inputs, equal weight.
    mixer.setMatrix({1.0, 1.0});

    std::array<double, 2> in{2.0, 5.0};
    std::array<double, 1> out{};

    mixer.processSample(in, out);
    mixer.processSample(in, out);

    EXPECT_NEAR(out[0], 7.0, k_eps);
}

TEST(MixerTest, ProcessBlockMatchesProcessSample) {
    constexpr std::size_t num_samples = 5;
    RTB::MatrixMixer<double, 1, 1> block_mixer;
    RTB::MatrixMixer<double, 1, 1> sample_mixer;
    block_mixer.prepare(44100.0, 0.0);
    sample_mixer.prepare(44100.0, 0.0);
    block_mixer.setGain(0, 0, 2.0);
    sample_mixer.setGain(0, 0, 2.0);

    std::array<double, num_samples> in_interleaved{};
    for (std::size_t i = 0; i < num_samples; ++i) {
        in_interleaved[i] = static_cast<double>(i + 1);
    }
    std::array<double, num_samples> out_block{};
    block_mixer.processBlock(
        in_interleaved.data(), out_block.data(), num_samples);

    std::array<double, num_samples> out_sample{};
    for (std::size_t i = 0; i < num_samples; ++i) {
        std::array<double, 1> in{in_interleaved[i]};
        std::array<double, 1> out{};
        sample_mixer.processSample(in, out);
        out_sample[i] = out[0];
    }

    for (std::size_t i = 0; i < num_samples; ++i) {
        EXPECT_NEAR(out_block[i], out_sample[i], k_eps);
    }
}

TEST(MixerTest, ProcessBlockPlanarMatchesProcessBlock) {
    constexpr std::size_t num_samples = 4;
    RTB::MatrixMixer<double, 2, 2> interleaved_mixer;
    RTB::MatrixMixer<double, 2, 2> planar_mixer;
    interleaved_mixer.prepare(44100.0, 0.0);
    planar_mixer.prepare(44100.0, 0.0);
    interleaved_mixer.setMatrix({1.0, 0.5, 0.5, 1.0});
    planar_mixer.setMatrix({1.0, 0.5, 0.5, 1.0});

    std::array<double, num_samples * 2> in_interleaved{
        1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0};
    std::array<double, num_samples * 2> out_interleaved{};
    interleaved_mixer.processBlock(
        in_interleaved.data(), out_interleaved.data(), num_samples);

    std::array<double, num_samples> in_ch0{1.0, 3.0, 5.0, 7.0};
    std::array<double, num_samples> in_ch1{2.0, 4.0, 6.0, 8.0};
    std::array<double, num_samples> out_ch0{};
    std::array<double, num_samples> out_ch1{};

    const std::array<const double*, 2> planar_inputs{in_ch0.data(),
                                                     in_ch1.data()};
    const std::array<double*, 2> planar_outputs{out_ch0.data(), out_ch1.data()};
    planar_mixer.processBlockPlanar(planar_inputs, planar_outputs, num_samples);

    for (std::size_t s = 0; s < num_samples; ++s) {
        EXPECT_NEAR(out_interleaved[s * 2 + 0], out_ch0[s], k_eps);
        EXPECT_NEAR(out_interleaved[s * 2 + 1], out_ch1[s], k_eps);
    }
}

// -----------------------------------------------------------------------------
// Death tests (Debug-only asserts)
// -----------------------------------------------------------------------------

#ifndef NDEBUG

TEST(MixerTest, SetGainBeforePrepareTriggersAssert) {
    EXPECT_DEATH(
        ([]() {
            RTB::MatrixMixer<double, 1, 1> mixer;  // prepare() never called
            mixer.setGain(0, 0, 1.0);
        }()),
        "");
}

TEST(MixerTest, SetGainOutOfBoundsTriggersAssert) {
    EXPECT_DEATH(([]() {
                     RTB::MatrixMixer<double, 1, 1> mixer;
                     mixer.prepare(44100.0, 10.0);
                     mixer.setGain(5, 0, 1.0);
                 }()),
                 "");
}

#endif  // NDEBUG
