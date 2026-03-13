cmake_minimum_required(VERSION 3.21)
project(RTB VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build/_exec)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build/lib)

include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_SOURCE_DIR}/lib)

# ==============================
# Google Highway
# ==============================
set(HWY_ENABLE_TESTS OFF CACHE BOOL "" FORCE)
set(HWY_ENABLE_EXAMPLES OFF CACHE BOOL "" FORCE)
set(HWY_ENABLE_CONTRIB OFF CACHE BOOL "" FORCE)
add_subdirectory(${CMAKE_SOURCE_DIR}/lib/highway)

# ==============================
# Sources + executable
# ==============================
file(GLOB_RECURSE SOURCES "${CMAKE_SOURCE_DIR}/src/*.cpp")

if(DEFINED ENV{CMAKE_PRESET_NAME})
    set(PRESET_NAME $ENV{CMAKE_PRESET_NAME})
elseif(DEFINED CMAKE_BUILD_TYPE)
    set(PRESET_NAME ${CMAKE_BUILD_TYPE})
else()
    set(PRESET_NAME "default")
endif()

add_executable(${PROJECT_NAME}_${PRESET_NAME} ${SOURCES})

target_link_libraries(${PROJECT_NAME}_${PRESET_NAME} PRIVATE hwy)

# Highway needs to know it can emit AVX2/FMA instructions.
# It handles runtime dispatch internally — these flags just unlock
# the instruction sets at the compiler level.
if(MSVC)
    target_compile_options(${PROJECT_NAME}_${PRESET_NAME} PRIVATE /arch:AVX2)
else()
    target_compile_options(${PROJECT_NAME}_${PRESET_NAME} PRIVATE -mavx2 -mfma)
endif()

# ==============================
# Testing
# ==============================
option(ENABLE_TESTING "Enable building of tests" OFF)

if(CMAKE_PRESET_NAME STREQUAL "test")
    message(STATUS "Detected 'test' preset: enabling tests")
    set(ENABLE_TESTING ON)
endif()

if(ENABLE_TESTING)
    enable_testing()
    add_subdirectory(lib/GoogleTest)
    add_subdirectory(test)
endif()
