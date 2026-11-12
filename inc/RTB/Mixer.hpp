/*
 ____ _____ ____
|  _ \_   _| __ )
| |_) || | |  _ \
|  _ < | | | |_) |
|_| \_\|_| |____/

 Mixer.hpp
 SIMD-accelerated matrix mixer (summing matrix) for multichannel audio.

 Copyright (c) 2026 Robin Breugelmans
 Licensed under the MIT License. See LICENSE in the project root.
*/

#ifndef RTB_MIXER_HPP
#define RTB_MIXER_HPP

// STL
#include <algorithm>
#include <array>
#include <cassert>
#include <cstddef>
#include <cstring>
#include <type_traits>

// LIB
#include <hwy/aligned_allocator.h>  // HWY_ALIGNMENT
#include <hwy/highway.h>

HWY_BEFORE_NAMESPACE();

namespace RTB {
namespace HWY_NAMESPACE {

namespace hn = hwy::HWY_NAMESPACE;

/**
 * @brief General-purpose matrix mixer (summing matrix).
 *
 * Maps Inputs audio inputs to Outputs audio outputs via a gain matrix G,
 * where:
 *
 *   output[i] = sum_j( G(i,j) * input[j] )
 *
 * Storage is row-major: G(i,j) = m_current[i * Inputs + j].
 * This means each row (one output's gains across all inputs) is contiguous
 * in memory, which is optimal for the inner SIMD loop.
 *
 * Gain changes are smoothed over a configurable ramp time to avoid clicks.
 * Call prepare() before processing to set the sample rate and ramp time.
 *
 * SIMD dispatch is handled by Google Highway — the best available instruction
 * set (AVX2, SSE4, NEON, etc.) is selected at runtime.
 *
 * @tparam T       Scalar type. float or double recommended.
 * @tparam Inputs  Number of input channels.
 * @tparam Outputs Number of output channels.
 */
template <typename T, std::size_t Inputs, std::size_t Outputs>
class MatrixMixer {
    static_assert(std::is_floating_point_v<T>,
                  "MatrixMixer<T>: T must be a floating-point type. "
                  "Integer mixing is not supported due to smoothing.");
    static_assert(Inputs > 0, "MatrixMixer: Inputs must be > 0.");
    static_assert(Outputs > 0, "MatrixMixer: Outputs must be > 0.");

    static constexpr std::size_t matrix_size = Inputs * Outputs;

   public:
    MatrixMixer() {
        m_current.fill(static_cast<T>(0));
        m_target.fill(static_cast<T>(0));
        m_delta.fill(static_cast<T>(0));
        m_ramp_samples_remaining = 0;
        m_ramp_samples = 0;
        m_sample_rate = static_cast<T>(44100);
    }

    // -------------------------------------------------------------------------
    // Setup
    // -------------------------------------------------------------------------

    /**
     * @brief Prepares the mixer for processing.
     *
     * Must be called before any process() call, and again if the sample
     * rate changes.
     *
     * @param sample_rate  Sample rate in Hz.
     * @param ramp_time_ms Gain ramp time in milliseconds. Gain changes are
     *                     linearly interpolated over this duration to
     *                     prevent clicks. Typical values: 5-20 ms.
     */
    void prepare(T sample_rate, T ramp_time_ms) {
        assert(sample_rate > static_cast<T>(0));
        assert(ramp_time_ms >= static_cast<T>(0));
        m_sample_rate = sample_rate;
        m_ramp_samples =
            static_cast<int>(sample_rate * ramp_time_ms / static_cast<T>(1000));
        m_ramp_samples = std::max(m_ramp_samples, 1);  // at least 1 sample
    }

    // -------------------------------------------------------------------------
    // Gain control
    // -------------------------------------------------------------------------

    /**
     * @brief Sets a single gain coefficient and triggers a ramp.
     *
     * @param output_idx  Output channel index [0, Outputs).
     * @param input_idx   Input channel index  [0, Inputs).
     * @param gain        Target gain value.
     */
    void setGain(std::size_t output_idx, std::size_t input_idx, T gain) {
        assert(output_idx < Outputs);
        assert(input_idx < Inputs);
        m_target[index(output_idx, input_idx)] = gain;
        recomputeDeltas();
    }

    /**
     * @brief Replaces the entire gain matrix and triggers a ramp.
     *
     * @param gains  Row-major array of size Outputs * Inputs.
     *               gains[i * Inputs + j] is the gain from input j to output i.
     */
    void setMatrix(const std::array<T, matrix_size>& gains) {
        m_target = gains;
        recomputeDeltas();
    }

    /**
     * @brief Returns the current (possibly mid-ramp) gain at (output_idx,
     * input_idx).
     */
    [[nodiscard]] T getGain(std::size_t output_idx,
                            std::size_t input_idx) const {
        assert(output_idx < Outputs);
        assert(input_idx < Inputs);
        return m_current[index(output_idx, input_idx)];
    }

    /**
     * @brief Returns the target gain matrix as a row-major array.
     */
    [[nodiscard]] const std::array<T, matrix_size>& getMatrix() const {
        return m_target;
    }

    /**
     * @brief Returns true if a gain ramp is currently in progress.
     */
    [[nodiscard]] bool isRamping() const {
        return m_ramp_samples_remaining > 0;
    }

    // -------------------------------------------------------------------------
    // Processing
    // -------------------------------------------------------------------------

    /**
     * @brief Processes a single sample.
     *
     * @param inputs   Array of Inputs input samples.
     * @param outputs  Array of Outputs output samples (overwritten).
     */
    void processSample(const std::array<T, Inputs>& inputs,
                       std::array<T, Outputs>& outputs) {
        advanceRamp();
        processFrameSIMD(inputs.data(), outputs.data());
    }

    /**
     * @brief Processes a block of samples.
     *
     * @param inputs       Pointer to interleaved input samples:
     *                     inputs[sample * Inputs + channel].
     * @param outputs      Pointer to interleaved output samples (overwritten).
     * @param num_samples  Number of samples to process.
     */
    void processBlock(const T* inputs, T* outputs, std::size_t num_samples) {
        assert(inputs != nullptr);
        assert(outputs != nullptr);

        for (std::size_t sample = 0; sample < num_samples; ++sample) {
            advanceRamp();

            const T* in_frame = inputs + (sample * Inputs);
            T* out_frame = outputs + (sample * Outputs);

            processFrameSIMD(in_frame, out_frame);
        }
    }

    /**
     * @brief Processes a block of samples with non-interleaved (planar) layout.
     *
     * @param inputs       Array of Inputs pointers, each pointing to
     *                     num_samples samples for that input channel.
     * @param outputs      Array of Outputs pointers (overwritten).
     * @param num_samples  Number of samples to process.
     */
    void processBlockPlanar(const std::array<const T*, Inputs>& inputs,
                            const std::array<T*, Outputs>& outputs,
                            std::size_t num_samples) {
        for (const T* channel_ptr : inputs) {
            assert(channel_ptr != nullptr);
        }
        for (const T* channel_ptr : outputs) {
            assert(channel_ptr != nullptr);
        }

        // Temp frame buffers for per-sample dispatch
        std::array<T, Inputs> in_frame;
        std::array<T, Outputs> out_frame;

        for (std::size_t sample = 0; sample < num_samples; ++sample) {
            advanceRamp();

            for (std::size_t channel = 0; channel < Inputs; ++channel) {
                in_frame[channel] = inputs[channel][sample];
            }

            processFrameSIMD(in_frame.data(), out_frame.data());

            for (std::size_t channel = 0; channel < Outputs; ++channel) {
                outputs[channel][sample] = out_frame[channel];
            }
        }
    }

   private:
    // -------------------------------------------------------------------------
    // Internal helpers
    // -------------------------------------------------------------------------

    static constexpr std::size_t index(std::size_t output_idx,
                                       std::size_t input_idx) {
        return (output_idx * Inputs) + input_idx;  // row-major
    }

    void recomputeDeltas() {
        assert(m_ramp_samples > 0 &&
               "MatrixMixer::recomputeDeltas() called before prepare(); "
               "call prepare() before setGain()/setMatrix().");

        if (m_ramp_samples <= 0) {
            // Not prepared yet: snap directly to target rather than
            // dividing by zero.
            m_current = m_target;
            m_ramp_samples_remaining = 0;
            return;
        }

        m_ramp_samples_remaining = m_ramp_samples;
        for (std::size_t i = 0; i < matrix_size; ++i) {
            m_delta[i] =
                (m_target[i] - m_current[i]) / static_cast<T>(m_ramp_samples);
        }
    }

    void advanceRamp() {
        if (m_ramp_samples_remaining <= 0) {
            return;
        }

        for (std::size_t i = 0; i < matrix_size; ++i) {
            m_current[i] += m_delta[i];
        }

        --m_ramp_samples_remaining;

        // Snap to target on completion to avoid floating-point drift
        if (m_ramp_samples_remaining == 0) {
            m_current = m_target;
        }
    }

    /**
     * @brief Core matrix-vector multiply for one frame using Highway SIMD.
     *
     * For each output i, accumulates:
     *   output[i] = sum_j( G(i,j) * input[j] )
     *
     * The inner loop over j is vectorised. Inputs are loaded in SIMD-width
     * chunks; the horizontal sum of the accumulator gives the scalar output.
     */
    void processFrameSIMD(const T* HWY_RESTRICT in_frame,
                          T* HWY_RESTRICT out_frame) const {
        const hn::ScalableTag<T> tag;
        const std::size_t lanes = hn::Lanes(tag);

        for (std::size_t i = 0; i < Outputs; ++i) {
            const T* row = m_current.data() + (i * Inputs);

            auto acc = hn::Zero(tag);
            std::size_t col = 0;

            // Vectorised inner loop — process `lanes` inputs at a time
            for (; col + lanes <= Inputs; col += lanes) {
                const auto gains = hn::LoadU(tag, row + col);
                const auto inputs_vec = hn::LoadU(tag, in_frame + col);
                acc = hn::MulAdd(gains, inputs_vec, acc);
            }

            // Horizontal sum of SIMD accumulator
            T sum = hn::ReduceSum(tag, acc);

            // Scalar tail for remaining inputs
            for (; col < Inputs; ++col) {
                sum += row[col] * in_frame[col];
            }

            out_frame[i] = sum;
        }
    }

    // -------------------------------------------------------------------------
    // State
    // -------------------------------------------------------------------------

    alignas(HWY_ALIGNMENT) std::array<T, matrix_size> m_current;  // live gains
    alignas(HWY_ALIGNMENT) std::array<T, matrix_size> m_target;  // target gains
    alignas(HWY_ALIGNMENT)
        std::array<T, matrix_size> m_delta;  // per-sample increment

    int m_ramp_samples_remaining;
    int m_ramp_samples;
    T m_sample_rate;
};

}  // namespace HWY_NAMESPACE
}  // namespace RTB

HWY_AFTER_NAMESPACE();

// ==============================
// Highway dynamic dispatch
// ==============================
//
// Re-export the best available implementation into RTB:: directly.
// Highway selects the optimal SIMD target (AVX2, SSE4, NEON, etc.)
// at runtime via HWY_EXPORT / HWY_DYNAMIC_DISPATCH if needed.
//
// For most use cases, including this header and constructing
// RTB::MatrixMixer<float, Inputs, Outputs> is sufficient — Highway
// picks the best target at compile time when HWY_COMPILE_ONLY_STATIC
// is defined, or at runtime otherwise.

namespace RTB {
using HWY_NAMESPACE::MatrixMixer;
}

#endif  // RTB_MIXER_HPP
