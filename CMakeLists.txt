cmake_minimum_required(VERSION 3.21)
project(RTB VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_EXTENSIONS OFF)

# ==============================
# Google Highway
# ==============================
set(HWY_ENABLE_TESTS OFF CACHE BOOL "" FORCE)
set(HWY_ENABLE_EXAMPLES OFF CACHE BOOL "" FORCE)
set(HWY_ENABLE_CONTRIB OFF CACHE BOOL "" FORCE)
add_subdirectory(${CMAKE_SOURCE_DIR}/lib/highway)

# ==============================
# RTB interface library
# ==============================
add_library(RTB INTERFACE)
target_include_directories(RTB INTERFACE ${CMAKE_SOURCE_DIR}/inc)
target_link_libraries(RTB INTERFACE hwy)

# ==============================
# Only build executable + tests when RTB is the root project
# ==============================
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)

    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build/_exec)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build/lib)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build/lib)

    if(DEFINED ENV{CMAKE_PRESET_NAME})
        set(PRESET_NAME $ENV{CMAKE_PRESET_NAME})
    elseif(DEFINED CMAKE_BUILD_TYPE)
        set(PRESET_NAME ${CMAKE_BUILD_TYPE})
    else()
        set(PRESET_NAME "default")
    endif()

    file(GLOB_RECURSE SOURCES "${CMAKE_SOURCE_DIR}/src/*.cpp")
    add_executable(${PROJECT_NAME}_${PRESET_NAME} ${SOURCES})

    target_link_libraries(${PROJECT_NAME}_${PRESET_NAME} PRIVATE RTB)

    if(MSVC)
        target_compile_options(${PROJECT_NAME}_${PRESET_NAME} PRIVATE /arch:AVX2)
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
        target_compile_options(${PROJECT_NAME}_${PRESET_NAME} PRIVATE -mavx2 -mfma)
    endif()

    # ==============================
    # Testing
    # ==============================
    option(ENABLE_TESTING "Enable building of tests" OFF)

    if(PRESET_NAME STREQUAL "test")
        message(STATUS "Detected 'test' preset: enabling tests")
        set(ENABLE_TESTING ON)
    endif()

    if(ENABLE_TESTING)
        enable_testing()
        add_subdirectory(lib/GoogleTest)
        add_subdirectory(test)
    endif()

endif()
