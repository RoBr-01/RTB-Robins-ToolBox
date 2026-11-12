/*
 ____ _____ ____
|  _ \_   _| __ )
| |_) || | |  _ \
|  _ < | | | |_) |
|_| \_\|_| |____/

 Plane_bench.cpp
 Benchmarks for RTB::Plane construction and intersection.

 Copyright (c) 2026 Robin Breugelmans
 Licensed under the MIT License. See LICENSE in the project root.
*/

#include <benchmark/benchmark.h>

#include <RTB/Plane.hpp>

namespace {

void BM_PlaneFromThreePoints(benchmark::State& state) {
    const RTB::Point<float, 3> point_a{0.0F, 0.0F, 0.0F};
    const RTB::Point<float, 3> point_b{1.0F, 0.0F, 0.0F};
    const RTB::Point<float, 3> point_c{0.0F, 1.0F, 0.0F};
    for (auto _ : state) {
        RTB::Plane<float> plane(point_a, point_b, point_c);
        benchmark::DoNotOptimize(plane);
    }
}
BENCHMARK(BM_PlaneFromThreePoints);

void BM_PlaneGetIntersection(benchmark::State& state) {
    const RTB::Plane<float> plane(0.0F, 0.0F, 1.0F, 0.0F);
    const RTB::Ray<float, 3> ray{RTB::Point<float, 3>{0.0F, 0.0F, 5.0F},
                                 RTB::Vector<float, 3>{0.0F, 0.0F, -1.0F}};
    for (auto _ : state) {
        auto result = plane.getIntersection(ray);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_PlaneGetIntersection);

void BM_PlaneReflect(benchmark::State& state) {
    RTB::Plane<float> plane(0.0F, 0.0F, 1.0F, 0.0F);
    for (auto _ : state) {
        RTB::Ray<float, 3> ray{RTB::Point<float, 3>{0.0F, 0.0F, 5.0F},
                               RTB::Vector<float, 3>{1.0F, 0.0F, -1.0F}};
        plane.reflect(ray, 5.0F);
        benchmark::DoNotOptimize(ray);
    }
}
BENCHMARK(BM_PlaneReflect);

void BM_IntersectPlanes(benchmark::State& state) {
    const RTB::Plane<float> plane_1(0.0F, 0.0F, 1.0F, 0.0F);
    const RTB::Plane<float> plane_2(1.0F, 0.0F, 0.0F, 0.0F);
    for (auto _ : state) {
        auto result = RTB::intersectPlanes(plane_1, plane_2);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_IntersectPlanes);

}  // namespace

BENCHMARK_MAIN();
