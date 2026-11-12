/*
 ____ _____ ____
|  _ \_   _| __ )
| |_) || | |  _ \
|  _ < | | | |_) |
|_| \_\|_| |____/

 Point_bench.cpp
 Benchmarks for RTB::Point free functions.

 Copyright (c) 2026 Robin Breugelmans
 Licensed under the MIT License. See LICENSE in the project root.
*/

#include <benchmark/benchmark.h>

#include <RTB/Point.hpp>

namespace {

void BM_PointDistance(benchmark::State& state) {
    RTB::Point<float, 3> lhs{1.0F, 2.0F, 3.0F};
    RTB::Point<float, 3> rhs{4.0F, 5.0F, 6.0F};
    for (auto _ : state) {
        auto result = RTB::distance2Points(lhs, rhs);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_PointDistance);

void BM_PointMidpoint(benchmark::State& state) {
    RTB::Point<float, 3> lhs{1.0F, 2.0F, 3.0F};
    RTB::Point<float, 3> rhs{4.0F, 5.0F, 6.0F};
    for (auto _ : state) {
        auto result = RTB::midpoint2Points(lhs, rhs);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_PointMidpoint);

}  // namespace

BENCHMARK_MAIN();
