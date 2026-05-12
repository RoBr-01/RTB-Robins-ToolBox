cmake_minimum_required(VERSION 4.3.2)
project(RTB VERSION 0.0.1)

include(CTest)
include(cmake/Checks.cmake)
include(cmake/CompileOptions.cmake)

CHECK_GIT_SUBMODULES()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Since we are header-only, expose as such
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/inc)

set_target_properties(${PROJECT_NAME} PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

target_link_libraries(${PROJECT_NAME} INTERFACE compile_options)

# Make sure we don't link hwy twice
if(NOT TARGET hwy)
    set(HWY_ENABLE_TESTS OFF CACHE BOOL "" FORCE)
    set(HWY_ENABLE_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(HWY_ENABLE_CONTRIB OFF CACHE BOOL "" FORCE)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/lib/highway)
endif()

target_link_libraries(${PROJECT_NAME} INTERFACE hwy)
target_include_directories(${PROJECT_NAME} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/lib/highway/hwy)


# Only build tests/bench/examples when we are the top-level project
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)

    option(${PROJECT_NAME}_ENABLE_TESTING "Enable building of tests" OFF)
    option(${PROJECT_NAME}_ENABLE_BENCHMARK "Enable building of benchmarks" OFF)
    option(${PROJECT_NAME}_ENABLE_EXAMPLES "Enable building of examples" OFF)

    if(${PROJECT_NAME}_ENABLE_TESTING)
        message(STATUS "Building Unit Tests")
        enable_testing()
        add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/test)
    endif()

    if(${PROJECT_NAME}_ENABLE_BENCHMARK)
        message(STATUS "Building Benchmarks")
        add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/bench)
    endif()

    if(${PROJECT_NAME}_ENABLE_EXAMPLES)
        message(STATUS "Building Examples")
        add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/examples)
    endif()

endif()
