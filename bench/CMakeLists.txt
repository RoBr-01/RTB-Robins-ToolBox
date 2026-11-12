set(BENCHMARKS
    Colour_bench
    Ellipse_bench
    Ellipsoid_bench
    Math_bench
    Mixer_bench
    Plane_bench
    Point_bench
    Random_bench
    Ray_bench
    Vector_bench
)

set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "" FORCE)
set(BENCHMARK_ENABLE_GTEST_TESTS OFF CACHE BOOL "" FORCE)
set(BENCHMARK_ENABLE_LIBPFM OFF CACHE BOOL "" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../lib/googlebenchmark ${CMAKE_CURRENT_BINARY_DIR}/../lib/googlebenchmark)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/../bin)

foreach(BENCH ${BENCHMARKS})
    add_executable(${BENCH} ${BENCH}.cpp)

    set_target_properties(${BENCH} PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS OFF
    )

    target_link_libraries(${BENCH} PRIVATE
        ${PROJECT_NAME}
        benchmark_main
        compile_options
    )

endforeach()
