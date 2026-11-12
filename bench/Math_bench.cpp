/*
 ____ _____ ____
|  _ \_   _| __ )
| |_) || | |  _ \
|  _ < | | | |_) |
|_| \_\|_| |____/

 Math_bench.cpp
 Benchmarks for RTB::Math trigonometric/coordinate helpers.

 Copyright (c) 2026 Robin Breugelmans
 Licensed under the MIT License. See LICENSE in the project root.
*/

#include <benchmark/benchmark.h>

#include <RTB/Math.hpp>

namespace {

void BM_NextPowerOf2(benchmark::State& state) {
    size_t number = 12345;
    for (auto _ : state) {
        auto result = RTB::nextPowerOf2(number);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_NextPowerOf2);

void BM_Cart2Sph(benchmark::State& state) {
    std::array<float, 3> cart{3.0F, 4.0F, 5.0F};
    for (auto _ : state) {
        auto result = RTB::cart2sph<float>(cart);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_Cart2Sph);

void BM_Sph2Cart(benchmark::State& state) {
    std::array<float, 3> sph{45.0F, 30.0F, 5.0F};
    for (auto _ : state) {
        auto result = RTB::sph2cart<float>(sph);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_Sph2Cart);

}  // namespace

BENCHMARK_MAIN();
