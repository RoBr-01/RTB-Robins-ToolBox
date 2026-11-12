/*
 ____ _____ ____
|  _ \_   _| __ )
| |_) || | |  _ \
|  _ < | | | |_) |
|_| \_\|_| |____/

 Ellipsoid_bench.cpp
 Benchmarks for RTB::Ellipsoid intersection and path-tracing operations.

 Copyright (c) 2026 Robin Breugelmans
 Licensed under the MIT License. See LICENSE in the project root.
*/

#include <benchmark/benchmark.h>

#include <RTB/Ellipsoid.hpp>
#include <vector>

namespace {

// Every benchmark below cycles through a precomputed table of varying
// inputs rather than reusing a single fixed input. A fixed, compile-time
// constant input lets -O3/-flto prove the result ahead of time and hoist
// the call out of the loop entirely — DoNotOptimize() only prevents the
// *result* from being discarded, not the *computation* from being folded
// away if every input was already known at compile time. Cycling through
// a runtime-sized, runtime-filled table defeats that.
constexpr std::size_t kTableSize = 1024;

std::vector<float> makeAngleTable(float start, float step) {
    std::vector<float> table(kTableSize);
    for (std::size_t i = 0; i < kTableSize; ++i) {
        table[i] = start + step * static_cast<float>(i);
    }
    return table;
}

std::vector<RTB::Point<float, 3>> makeSurfacePointTable(
    const RTB::Ellipsoid<float>& ellipsoid, float elevation) {
    std::vector<RTB::Point<float, 3>> table;
    table.reserve(kTableSize);
    for (std::size_t i = 0; i < kTableSize; ++i) {
        const float azimuth =
            360.0F * static_cast<float>(i) / static_cast<float>(kTableSize);
        const float radius = ellipsoid.getRadiusAt(azimuth, elevation);
        const auto cart = RTB::sph2cart<float>(
            std::array<float, 3>{azimuth, elevation, radius});
        table.emplace_back(cart[0], cart[1], cart[2]);
    }
    return table;
}

void BM_EllipsoidGetRadiusAt(benchmark::State& state) {
    const RTB::Ellipsoid<float> ellipsoid(3.0F, 2.0F, 1.0F);
    const auto azimuths = makeAngleTable(0.0F, 0.33F);

    std::size_t i = 0;
    for (auto _ : state) {
        auto result = ellipsoid.getRadiusAt(azimuths[i % kTableSize], 20.0F);
        benchmark::DoNotOptimize(result);
        ++i;
    }
}
BENCHMARK(BM_EllipsoidGetRadiusAt);

void BM_EllipsoidIntersectRay(benchmark::State& state) {
    const RTB::Ellipsoid<float> ellipsoid(3.0F, 2.0F, 1.0F);
    const auto azimuths = makeAngleTable(0.0F, 0.33F);

    std::vector<RTB::Ray<float, 3>> rays;
    rays.reserve(kTableSize);
    for (float azimuth : azimuths) {
        const RTB::Point<float, 3> origin{-10.0F, 0.0F, 0.0F};
        const auto cart =
            RTB::sph2cart<float>(std::array<float, 3>{azimuth, 0.0F, 1.0F});
        rays.emplace_back(origin,
                          RTB::Vector<float, 3>{cart[0], cart[1], cart[2]});
    }

    std::size_t i = 0;
    for (auto _ : state) {
        auto result = ellipsoid.intersectRay(rays[i % kTableSize]);
        benchmark::DoNotOptimize(result);
        ++i;
    }
}
BENCHMARK(BM_EllipsoidIntersectRay);

void BM_EllipsoidIntersectPlane(benchmark::State& state) {
    const RTB::Ellipsoid<float> ellipsoid(3.0F, 2.0F, 1.0F);
    const auto azimuths = makeAngleTable(0.0F, 0.33F);

    std::vector<RTB::Plane<float>> planes;
    planes.reserve(kTableSize);
    for (float azimuth : azimuths) {
        const auto cart =
            RTB::sph2cart<float>(std::array<float, 3>{azimuth, 10.0F, 1.0F});
        RTB::Plane<float> plane(cart[0], cart[1], cart[2], -0.4F);
        plane.normalize();
        planes.push_back(plane);
    }

    std::size_t i = 0;
    for (auto _ : state) {
        auto result = ellipsoid.intersectPlane(planes[i % kTableSize]);
        benchmark::DoNotOptimize(result);
        ++i;
    }
}
BENCHMARK(BM_EllipsoidIntersectPlane);

void BM_EllipsoidArcLength(benchmark::State& state) {
    const RTB::Ellipsoid<float> ellipsoid(3.0F, 2.0F, 1.0F);
    const auto points_1 = makeSurfacePointTable(ellipsoid, 0.0F);
    const auto points_2 = makeSurfacePointTable(ellipsoid, 15.0F);

    std::size_t i = 0;
    for (auto _ : state) {
        auto result = ellipsoid.arcLength(points_1[i % kTableSize],
                                          points_2[i % kTableSize]);
        benchmark::DoNotOptimize(result);
        ++i;
    }
}
BENCHMARK(BM_EllipsoidArcLength);

void BM_EllipsoidTracePathUnoccluded(benchmark::State& state) {
    const RTB::Ellipsoid<float> ellipsoid(3.0F, 2.0F, 1.0F);
    const auto points_on = makeSurfacePointTable(ellipsoid, 0.0F);

    // Sources placed far along each surface point's own outward normal
    // direction are always directly visible (unoccluded) from that point.
    std::vector<RTB::Point<float, 3>> points_outside;
    points_outside.reserve(kTableSize);
    for (const auto& point_on : points_on) {
        RTB::Vector<float, 3> normal_dir{
            point_on[0] / 9.0F, point_on[1] / 4.0F, point_on[2]};
        normal_dir.normalizeInPlace();
        points_outside.emplace_back(point_on[0] + normal_dir[0] * 10.0F,
                                    point_on[1] + normal_dir[1] * 10.0F,
                                    point_on[2] + normal_dir[2] * 10.0F);
    }

    std::size_t i = 0;
    for (auto _ : state) {
        auto result = ellipsoid.tracePath(points_outside[i % kTableSize],
                                          points_on[i % kTableSize]);
        benchmark::DoNotOptimize(result);
        ++i;
    }
}
BENCHMARK(BM_EllipsoidTracePathUnoccluded);

void BM_EllipsoidTracePathOccluded(benchmark::State& state) {
    const RTB::Ellipsoid<float> ellipsoid(3.0F, 2.0F, 1.0F);
    const auto points_on = makeSurfacePointTable(ellipsoid, 0.0F);

    // A single fixed source on the far side of the ellipsoid from every
    // table point on the opposite hemisphere keeps each pair genuinely
    // occluded while still varying point_on across the table.
    const RTB::Point<float, 3> point_outside{0.0F, 10.0F, 0.0F};

    std::size_t i = 0;
    for (auto _ : state) {
        auto result =
            ellipsoid.tracePath(point_outside, points_on[i % kTableSize]);
        benchmark::DoNotOptimize(result);
        ++i;
    }
}
BENCHMARK(BM_EllipsoidTracePathOccluded);

}  // namespace

BENCHMARK_MAIN();
