/*
 ____ _____ ____
|  _ \_   _| __ )
| |_) || | |  _ \
|  _ < | | | |_) |
|_| \_\|_| |____/

 Ray_bench.cpp
 Benchmarks for RTB::Ray operations.

 Copyright (c) 2026 Robin Breugelmans
 Licensed under the MIT License. See LICENSE in the project root.
*/

#include <benchmark/benchmark.h>

#include <RTB/Ray.hpp>

namespace {

void BM_RayGetPosition(benchmark::State& state) {
    RTB::Ray<float, 3> ray{RTB::Point<float, 3>{0.0F, 0.0F, 0.0F},
                           RTB::Vector<float, 3>{1.0F, 0.0F, 0.0F}};
    for (auto _ : state) {
        auto result = ray.getPosition(3.5F);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_RayGetPosition);

void BM_RayNormalize(benchmark::State& state) {
    for (auto _ : state) {
        RTB::Ray<float, 3> ray{RTB::Point<float, 3>{0.0F, 0.0F, 0.0F},
                               RTB::Vector<float, 3>{3.0F, 4.0F, 12.0F}};
        ray.normalize();
        benchmark::DoNotOptimize(ray);
    }
}
BENCHMARK(BM_RayNormalize);

}  // namespace

BENCHMARK_MAIN();
