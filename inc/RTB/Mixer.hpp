#ifndef MATRIX_MIXER_HPP
#define MATRIX_MIXER_HPP

// STL
#include <algorithm>
#include <array>
#include <cassert>
#include <cstddef>
#include <cstring>
#include <type_traits>

// Google Highway — SIMD abstraction
// https://github.com/google/highway
// Requires: highway/highway.h on include path
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
 * Call Prepare() before processing to set the sample rate and ramp time.
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
    static_assert(std::is_floating_point<T>::value,
                  "MatrixMixer<T>: T must be a floating-point type. "
                  "Integer mixing is not supported due to smoothing.");
    static_assert(Inputs > 0, "MatrixMixer: Inputs must be > 0.");
    static_assert(Outputs > 0, "MatrixMixer: Outputs must be > 0.");

    static constexpr std::size_t kMatrixSize = Inputs * Outputs;

   public:
    MatrixMixer() {
        m_current.fill(static_cast<T>(0));
        m_target.fill(static_cast<T>(0));
        m_delta.fill(static_cast<T>(0));
        m_rampSamplesRemaining = 0;
        m_rampSamples = 0;
        m_sampleRate = static_cast<T>(44100);
    }

    // -------------------------------------------------------------------------
    // Setup
    // -------------------------------------------------------------------------

    /**
     * @brief Prepares the mixer for processing.
     *
     * Must be called before any Process() call, and again if the sample
     * rate changes.
     *
     * @param sampleRate  Sample rate in Hz.
     * @param rampTimeMs  Gain ramp time in milliseconds. Gain changes are
     *                    linearly interpolated over this duration to prevent
     *                    clicks. Typical values: 5–20 ms.
     */
    void Prepare(T sampleRate, T rampTimeMs) {
        assert(sampleRate > static_cast<T>(0));
        assert(rampTimeMs >= static_cast<T>(0));
        m_sampleRate = sampleRate;
        m_rampSamples =
            static_cast<int>(sampleRate * rampTimeMs / static_cast<T>(1000));
        m_rampSamples = std::max(m_rampSamples, 1);  // at least 1 sample
    }

    // -------------------------------------------------------------------------
    // Gain control
    // -------------------------------------------------------------------------

    /**
     * @brief Sets a single gain coefficient and triggers a ramp.
     *
     * @param outputIdx  Output channel index [0, Outputs).
     * @param inputIdx   Input channel index  [0, Inputs).
     * @param gain       Target gain value.
     */
    void SetGain(std::size_t outputIdx, std::size_t inputIdx, T gain) {
        assert(outputIdx < Outputs);
        assert(inputIdx < Inputs);
        m_target[Index(outputIdx, inputIdx)] = gain;
        RecomputeDeltas();
    }

    /**
     * @brief Replaces the entire gain matrix and triggers a ramp.
     *
     * @param gains  Row-major array of size Outputs * Inputs.
     *               gains[i * Inputs + j] is the gain from input j to output i.
     */
    void SetMatrix(const std::array<T, kMatrixSize>& gains) {
        m_target = gains;
        RecomputeDeltas();
    }

    /**
     * @brief Returns the current (possibly mid-ramp) gain at (outputIdx,
     * inputIdx).
     */
    T GetGain(std::size_t outputIdx, std::size_t inputIdx) const {
        assert(outputIdx < Outputs);
        assert(inputIdx < Inputs);
        return m_current[Index(outputIdx, inputIdx)];
    }

    /**
     * @brief Returns the target gain matrix as a row-major array.
     */
    const std::array<T, kMatrixSize>& GetMatrix() const {
        return m_target;
    }

    /**
     * @brief Returns true if a gain ramp is currently in progress.
     */
    bool IsRamping() const {
        return m_rampSamplesRemaining > 0;
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
    void ProcessSample(const std::array<T, Inputs>& inputs,
                       std::array<T, Outputs>& outputs) {
        AdvanceRamp();
        ProcessSampleWithCurrentMatrix(inputs, outputs);
    }

    /**
     * @brief Processes a block of samples.
     *
     * @param inputs      Pointer to interleaved input samples:
     *                    inputs[sample * Inputs + channel].
     * @param outputs     Pointer to interleaved output samples (overwritten).
     * @param numSamples  Number of samples to process.
     */
    void ProcessBlock(const T* inputs, T* outputs, std::size_t numSamples) {
        assert(inputs != nullptr);
        assert(outputs != nullptr);

        for (std::size_t s = 0; s < numSamples; ++s) {
            AdvanceRamp();

            const T* inFrame = inputs + s * Inputs;
            T* outFrame = outputs + s * Outputs;

            ProcessFrameSIMD(inFrame, outFrame);
        }
    }

    /**
     * @brief Processes a block of samples with non-interleaved (planar) layout.
     *
     * @param inputs      Array of Inputs pointers, each pointing to numSamples
     *                    samples for that input channel.
     * @param outputs     Array of Outputs pointers (overwritten).
     * @param numSamples  Number of samples to process.
     */
    void ProcessBlockPlanar(const T* const inputs[Inputs],
                            T* const outputs[Outputs],
                            std::size_t numSamples) {
        assert(inputs != nullptr);
        assert(outputs != nullptr);

        // Temp frame buffers for per-sample dispatch
        std::array<T, Inputs> inFrame;
        std::array<T, Outputs> outFrame;

        for (std::size_t s = 0; s < numSamples; ++s) {
            AdvanceRamp();

            for (std::size_t c = 0; c < Inputs; ++c)
                inFrame[c] = inputs[c][s];

            ProcessFrameSIMD(inFrame.data(), outFrame.data());

            for (std::size_t c = 0; c < Outputs; ++c)
                outputs[c][s] = outFrame[c];
        }
    }

   private:
    // -------------------------------------------------------------------------
    // Internal helpers
    // -------------------------------------------------------------------------

    static constexpr std::size_t Index(std::size_t outputIdx,
                                       std::size_t inputIdx) {
        return outputIdx * Inputs + inputIdx;  // row-major
    }

    void RecomputeDeltas() {
        m_rampSamplesRemaining = m_rampSamples;
        for (std::size_t i = 0; i < kMatrixSize; ++i)
            m_delta[i] =
                (m_target[i] - m_current[i]) / static_cast<T>(m_rampSamples);
    }

    void AdvanceRamp() {
        if (m_rampSamplesRemaining <= 0)
            return;

        for (std::size_t i = 0; i < kMatrixSize; ++i)
            m_current[i] += m_delta[i];

        --m_rampSamplesRemaining;

        // Snap to target on completion to avoid floating-point drift
        if (m_rampSamplesRemaining == 0)
            m_current = m_target;
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
    void ProcessFrameSIMD(const T* HWY_RESTRICT inFrame,
                          T* HWY_RESTRICT outFrame) const {
        const hn::ScalableTag<T> d;
        const std::size_t lanes = hn::Lanes(d);

        for (std::size_t i = 0; i < Outputs; ++i) {
            const T* row = m_current.data() + i * Inputs;

            auto acc = hn::Zero(d);
            std::size_t j = 0;

            // Vectorised inner loop — process `lanes` inputs at a time
            for (; j + lanes <= Inputs; j += lanes) {
                const auto g = hn::LoadU(d, row + j);
                const auto x = hn::LoadU(d, inFrame + j);
                acc = hn::MulAdd(g, x, acc);
            }

            // Horizontal sum of SIMD accumulator
            T sum = hn::ReduceSum(d, acc);

            // Scalar tail for remaining inputs
            for (; j < Inputs; ++j)
                sum += row[j] * inFrame[j];

            outFrame[i] = sum;
        }
    }

    // -------------------------------------------------------------------------
    // State
    // -------------------------------------------------------------------------

    alignas(HWY_ALIGNMENT) std::array<T, kMatrixSize> m_current;  // live gains
    alignas(HWY_ALIGNMENT) std::array<T, kMatrixSize> m_target;  // target gains
    alignas(HWY_ALIGNMENT)
        std::array<T, kMatrixSize> m_delta;  // per-sample increment

    int m_rampSamplesRemaining;
    int m_rampSamples;
    T m_sampleRate;
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

#endif  // MATRIX_MIXER_HPP