/*
 ____ _____ ____
|  _ \_   _| __ )
| |_) || | |  _ \
|  _ < | | | |_) |
|_| \_\|_| |____/

 Random_bench.cpp
 Benchmarks for RTB::Random sampling helpers.

 Copyright (c) 2026 Robin Breugelmans
 Licensed under the MIT License. See LICENSE in the project root.
*/

#include <benchmark/benchmark.h>

#include <RTB/Random.hpp>

namespace {

void BM_RandomValue(benchmark::State& state) {
    uint32_t seed = 12345U;
    for (auto _ : state) {
        auto result = RTB::randomValue(seed);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_RandomValue);

void BM_RandomHashValue(benchmark::State& state) {
    uint32_t seed = 12345U;
    for (auto _ : state) {
        auto result = RTB::randomHashValue(seed);
        benchmark::DoNotOptimize(result);
        ++seed;
    }
}
BENCHMARK(BM_RandomHashValue);

void BM_RandSampleSphere(benchmark::State& state) {
    std::mt19937 generator(42);
    for (auto _ : state) {
        auto result = RTB::randSampleSphere<float>(generator);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_RandSampleSphere);

void BM_RandSampleSpherePoint(benchmark::State& state) {
    std::mt19937 generator(42);
    for (auto _ : state) {
        auto result = RTB::randSampleSpherePoint<float>(generator);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_RandSampleSpherePoint);

}  // namespace

BENCHMARK_MAIN();
