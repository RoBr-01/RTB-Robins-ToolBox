/*
 ____ _____ ____
|  _ \_   _| __ )
| |_) || | |  _ \
|  _ < | | | |_) |
|_| \_\|_| |____/

 Ellipse_bench.cpp
 Benchmarks for RTB::Ellipse arc-length quadrature.

 Copyright (c) 2026 Robin Breugelmans
 Licensed under the MIT License. See LICENSE in the project root.
*/

#include <benchmark/benchmark.h>

#include <RTB/Ellipse.hpp>

namespace {

RTB::Ellipse<float> makeEllipse() {
    return RTB::Ellipse<float>{
        RTB::Point<float, 3>{0.0F, 0.0F, 0.0F},
        {RTB::Vector<float, 3>{1.0F, 0.0F, 0.0F},
         RTB::Vector<float, 3>{0.0F, 1.0F, 0.0F}},
        {5.0F, 2.0F},
        RTB::Vector<float, 3>{0.0F, 0.0F, 1.0F},
    };
}

void BM_EllipseCircumference(benchmark::State& state) {
    const auto ellipse = makeEllipse();
    for (auto _ : state) {
        auto result = ellipse.circumference();
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_EllipseCircumference);

void BM_EllipseArcLength(benchmark::State& state) {
    const auto ellipse = makeEllipse();
    const RTB::Point<float, 3> point_1{5.0F, 0.0F, 0.0F};
    const RTB::Point<float, 3> point_2{0.0F, 2.0F, 0.0F};
    for (auto _ : state) {
        auto result = ellipse.arcLength(point_1, point_2);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_EllipseArcLength);

}  // namespace

BENCHMARK_MAIN();
