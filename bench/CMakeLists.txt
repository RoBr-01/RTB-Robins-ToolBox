set(BENCHMARKS
)

set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "" FORCE)
set(BENCHMARK_ENABLE_GTEST_TESTS OFF CACHE BOOL "" FORCE)
set(BENCHMARK_ENABLE_LIBPFM OFF CACHE BOOL "" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../lib/googlebenchmark ${CMAKE_CURRENT_BINARY_DIR}/../lib/googlebenchmark)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/../bin)

foreach(BENCH ${BENCHMARKS})
    add_executable(${BENCH} ${BENCH}.cpp)

    target_link_libraries(${BENCH} PRIVATE
        ${PROJECT_NAME}
        benchmark_main
        compile_options
        sanitizers
    )

endforeach()
