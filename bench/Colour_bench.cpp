/*
 ____ _____ ____
|  _ \_   _| __ )
| |_) || | |  _ \
|  _ < | | | |_) |
|_| \_\|_| |____/

 Colour_bench.cpp
 Benchmarks for RTB::Colour operations.

 Copyright (c) 2026 Robin Breugelmans
 Licensed under the MIT License. See LICENSE in the project root.
*/

#include <benchmark/benchmark.h>

#include <RTB/Colour.hpp>

namespace {

void BM_ColourClamp(benchmark::State& state) {
    for (auto _ : state) {
        RTB::Colour8_3 colour{10, 200, 255};
        colour.clamp(20, 230);
        benchmark::DoNotOptimize(colour);
    }
}
BENCHMARK(BM_ColourClamp);

}  // namespace

BENCHMARK_MAIN();
