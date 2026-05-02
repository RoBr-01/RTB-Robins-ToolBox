cmake_minimum_required(VERSION 3.21)
project(RTB VERSION 0.0.1)

include(CTest)
include(cmake/Checks.cmake)

CHECK_GIT_SUBMODULES()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_EXTENSIONS OFF)

# Since we are header-only, expose as such
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE ${CMAKE_SOURCE_DIR}/inc)

# We do need Highway, so add that
set(HWY_ENABLE_TESTS OFF CACHE BOOL "" FORCE)
set(HWY_ENABLE_EXAMPLES OFF CACHE BOOL "" FORCE)
set(HWY_ENABLE_CONTRIB OFF CACHE BOOL "" FORCE)
add_subdirectory(${CMAKE_SOURCE_DIR}/lib/highway)
target_link_libraries(${PROJECT_NAME} INTERFACE hwy)
target_include_directories(${PROJECT_NAME} INTERFACE ${CMAKE_SOURCE_DIR}/lib/highway/hwy)

option(BUILD_TESTS "Build tests for RTB" OFF)

if(BUILD_TESTS)
    message("-- Building tests for RTB")

    # This just makes sure that CTest can find our tests
    enable_testing()

    set(TEST_NAME ${PROJECT_NAME}_Tests)
    add_subdirectory(${CMAKE_SOURCE_DIR}/test)
else()
    message("-- Not building tests for RTB")
endif()
