# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project overview

RTB ("Robin's ToolBox") is a C++17 header-only templated library for geometry and spatial-audio primitives (vectors, points, planes, ellipses/ellipsoids, rays, a matrix mixer, colour, timers, RNG). It uses Google Highway for SIMD. The whole library lives under [inc/RTB/](inc/RTB/) and is consumed by including [inc/RTB/RTB.hpp](inc/RTB/RTB.hpp) (everything) or individual headers.

## Build system

CMake + Ninja via presets (see [CMakePresets.json](CMakePresets.json)). The root [CMakeLists.txt](CMakeLists.txt) defines `RTB` as an `INTERFACE` library; tests/bench/examples/dev subdirectories are only built when RTB is the top-level project, gated by `RTB_ENABLE_TESTING`, `RTB_ENABLE_BENCHMARK`, `RTB_ENABLE_EXAMPLES`, `RTB_ENABLE_DEV`.

Configure + build with a preset:

```sh
cmake --preset test-debug
cmake --build --preset test-debug
```

Available presets (configure/build names match): `dev`, `lib-debug`, `lib-release`, `test-debug`, `test-release`, `bench-debug`, `bench-release`, `examples-debug`, `examples-release`.

Git submodules (`lib/highway`, `lib/googletest`, `lib/googlebenchmark`) are auto-initialized by `CHECK_GIT_SUBMODULES()` in [cmake/Checks.cmake](cmake/Checks.cmake) on every configure.

Compile flags live in [cmake/CompileOptions.cmake](cmake/CompileOptions.cmake): Debug enables heavy `-W*` warnings (treated informationally, not as errors); Release enables `-O3 -flto -ffast-math -fno-exceptions -fno-rtti` and strips symbols. A separate `sanitizers` interface target enables ASan/UBSan/bounds sanitizers in Debug and is linked into tests/bench/examples/dev.

## Running tests

Tests use GoogleTest and are discovered individually via `gtest_discover_tests` (see [test/CMakeLists.txt](test/CMakeLists.txt)). Test sources live in [test/](test/), one file per module (e.g. `Vector_test.cpp`, `Ellipsoid_test.cpp`).

```sh
cmake --preset test-debug
cmake --build --preset test-debug
ctest --test-dir build/test-debug
```

Run a single test binary directly (built to `build/test-debug/bin/`), or filter with gtest's `--gtest_filter`:

```sh
./build/test-debug/bin/Vector_test --gtest_filter=VectorTest.DotProduct
```

When adding a new test file, register it in the `TESTS` list in [test/CMakeLists.txt](test/CMakeLists.txt).

Benchmarks ([bench/](bench/)) and examples ([examples/](examples/)) follow the identical pattern — add the target name to the `BENCHMARKS`/`EXAMPLES` list in their respective `CMakeLists.txt`, backed by Google Benchmark.

## Documentation

Doxygen docs are generated from the root via `doxygen` (config in [Doxyfile](Doxyfile)), producing a `docs/` folder. Headers use Doxygen-style `/** @brief ... */` comments on public classes/functions — match this style when adding new public API.

## Code architecture

- **Templated, header-only, no `.cpp` implementation files for the library itself.** Every type is `template <typename T, std::size_t N>` (or similar), `static_assert`-checked for `std::is_arithmetic_v<T>`, and declared entirely in its header under `namespace RTB`.
- **Forward declaration block + free-function/operator declarations precede the class definition** in each header (see [inc/RTB/Vector.hpp](inc/RTB/Vector.hpp)) — operators (`+`, `-`, `*`, `==`, `<<`) and free functions (`crossProduct`, `dotProduct`) are non-member, declared before the class, defined after.
- **Dependency direction**: `Point` is a dependency of `Vector` (a `Vector` can be constructed from two `Point`s); `Ellipsoid`/`Ray`/`Plane` build on `Vector`/`Point`. Check existing includes in a header before assuming ordering.
- **SIMD via Highway**: modules that need vectorized math (e.g. [inc/RTB/Mixer.hpp](inc/RTB/Mixer.hpp)) wrap their implementation in `HWY_BEFORE_NAMESPACE()` / `namespace RTB { namespace HWY_NAMESPACE { ... } }` / `HWY_AFTER_NAMESPACE()`, aliasing `hn = hwy::HWY_NAMESPACE`. Follow this pattern exactly for any new SIMD code — it's what allows Highway to compile the same source per target instruction set.
- **Axis/direction conventions are centralized** in [inc/RTB/Standards.hpp](inc/RTB/Standards.hpp) (SOFA convention: +X front, +Y left, +Z up) along with physical constants (`speed_of_sound`, `speed_of_light`). Use these enums (`Axes`, `Directions`, `Vertices`) rather than magic indices/strings when referring to axes or spatial directions.
- **Naming**: classes `CamelCase`, functions `camelBack`, variables `lower_case` (enforced by [.clang-tidy](.clang-tidy)). Member variables use an `m_` prefix (e.g. `m_components`). Formatting is Google-based, 4-space indent, 80-column limit, no bin-packed arguments ([.clang-format](.clang-format)).
- **Testing convention**: one test file per header module, named `<Module>_test.cpp`, linked against the `RTB` interface target plus `gtest_main`, `compile_options`, and `sanitizers`.
