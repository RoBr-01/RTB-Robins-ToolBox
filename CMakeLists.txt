cmake_minimum_required(VERSION 3.21)
project(RTB VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build/_exec)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build/lib)

include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/lib)

file(GLOB_RECURSE SOURCES "${CMAKE_SOURCE_DIR}/src/*.cpp")

if(DEFINED ENV{CMAKE_PRESET_NAME})
    set(PRESET_NAME $ENV{CMAKE_PRESET_NAME})
elseif(DEFINED CMAKE_BUILD_TYPE)
    set(PRESET_NAME ${CMAKE_BUILD_TYPE})
else()
    set(PRESET_NAME "default")
endif()

# Now use that preset name in the executable target name
add_executable(${PROJECT_NAME}_${PRESET_NAME} ${SOURCES})

# Optional: testing
option(ENABLE_TESTING "Enable building of tests" OFF)

# Enable testing automatically when using "test" preset
if(CMAKE_PRESET_NAME STREQUAL "test")
    message(STATUS "Detected 'test' preset: enabling tests")
    set(ENABLE_TESTING ON)
endif()

if(ENABLE_TESTING)
    enable_testing()
    add_subdirectory(lib/GoogleTest)
    add_subdirectory(test)
endif()