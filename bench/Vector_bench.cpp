/*
 ____ _____ ____
|  _ \_   _| __ )
| |_) || | |  _ \
|  _ < | | | |_) |
|_| \_\|_| |____/

 Vector_bench.cpp
 Benchmarks for RTB::Vector arithmetic and geometric operations.

 Copyright (c) 2026 Robin Breugelmans
 Licensed under the MIT License. See LICENSE in the project root.
*/

#include <benchmark/benchmark.h>

#include <RTB/Vector.hpp>

namespace {

void BM_VectorAdd(benchmark::State& state) {
    RTB::Vector<float, 3> lhs{1.0F, 2.0F, 3.0F};
    RTB::Vector<float, 3> rhs{4.0F, 5.0F, 6.0F};
    for (auto _ : state) {
        auto result = lhs + rhs;
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_VectorAdd);

void BM_VectorDotProduct(benchmark::State& state) {
    RTB::Vector<float, 3> lhs{1.0F, 2.0F, 3.0F};
    RTB::Vector<float, 3> rhs{4.0F, 5.0F, 6.0F};
    for (auto _ : state) {
        auto result = RTB::dotProduct(lhs, rhs);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_VectorDotProduct);

void BM_VectorCrossProduct(benchmark::State& state) {
    RTB::Vector<float, 3> lhs{1.0F, 2.0F, 3.0F};
    RTB::Vector<float, 3> rhs{4.0F, 5.0F, 6.0F};
    for (auto _ : state) {
        auto result = RTB::crossProduct(lhs, rhs);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_VectorCrossProduct);

void BM_VectorMagnitude(benchmark::State& state) {
    RTB::Vector<float, 3> vector{3.0F, 4.0F, 0.0F};
    for (auto _ : state) {
        auto result = vector.magnitude();
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_VectorMagnitude);

void BM_VectorNormalize(benchmark::State& state) {
    RTB::Vector<float, 3> vector{3.0F, 4.0F, 12.0F};
    for (auto _ : state) {
        auto result = vector.normalize();
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_VectorNormalize);

}  // namespace

BENCHMARK_MAIN();
