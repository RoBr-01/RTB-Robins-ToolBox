/*
 ____ _____ ____
|  _ \_   _| __ )
| |_) || | |  _ \
|  _ < | | | |_) |
|_| \_\|_| |____/

 Mixer_bench.cpp
 Benchmarks for RTB::MatrixMixer SIMD processing.

 Copyright (c) 2026 Robin Breugelmans
 Licensed under the MIT License. See LICENSE in the project root.
*/

#include <benchmark/benchmark.h>

#include <RTB/Mixer.hpp>
#include <vector>

namespace {

template <std::size_t Inputs, std::size_t Outputs>
void runProcessSample(benchmark::State& state) {
    RTB::MatrixMixer<float, Inputs, Outputs> mixer;
    mixer.prepare(48000.0F, 10.0F);

    std::array<float, Inputs * Outputs> gains{};
    for (auto& gain : gains) {
        gain = 0.5F;
    }
    mixer.setMatrix(gains);

    std::array<float, Inputs> inputs{};
    for (auto& sample : inputs) {
        sample = 1.0F;
    }
    std::array<float, Outputs> outputs{};

    for (auto _ : state) {
        mixer.processSample(inputs, outputs);
        benchmark::DoNotOptimize(outputs);
    }
}

void BM_MixerProcessSample_8x8(benchmark::State& state) {
    runProcessSample<8, 8>(state);
}
BENCHMARK(BM_MixerProcessSample_8x8);

void BM_MixerProcessSample_16x2(benchmark::State& state) {
    runProcessSample<16, 2>(state);
}
BENCHMARK(BM_MixerProcessSample_16x2);

void BM_MixerProcessSample_64x64(benchmark::State& state) {
    runProcessSample<64, 64>(state);
}
BENCHMARK(BM_MixerProcessSample_64x64);

template <std::size_t Inputs, std::size_t Outputs>
void runProcessBlock(benchmark::State& state) {
    constexpr std::size_t block_size = 512;

    RTB::MatrixMixer<float, Inputs, Outputs> mixer;
    mixer.prepare(48000.0F, 10.0F);

    std::array<float, Inputs * Outputs> gains{};
    for (auto& gain : gains) {
        gain = 0.5F;
    }
    mixer.setMatrix(gains);

    std::vector<float> inputs(block_size * Inputs, 1.0F);
    std::vector<float> outputs(block_size * Outputs, 0.0F);

    for (auto _ : state) {
        mixer.processBlock(inputs.data(), outputs.data(), block_size);
        benchmark::DoNotOptimize(outputs);
    }

    state.SetItemsProcessed(static_cast<int64_t>(state.iterations()) *
                            static_cast<int64_t>(block_size));
}

void BM_MixerProcessBlock_8x8(benchmark::State& state) {
    runProcessBlock<8, 8>(state);
}
BENCHMARK(BM_MixerProcessBlock_8x8);

void BM_MixerProcessBlock_16x2(benchmark::State& state) {
    runProcessBlock<16, 2>(state);
}
BENCHMARK(BM_MixerProcessBlock_16x2);

void BM_MixerProcessBlock_64x64(benchmark::State& state) {
    runProcessBlock<64, 64>(state);
}
BENCHMARK(BM_MixerProcessBlock_64x64);

}  // namespace

BENCHMARK_MAIN();
