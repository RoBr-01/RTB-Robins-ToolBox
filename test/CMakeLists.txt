set(TESTS
    Ellipsoid_test
    Math_test
    Plane_test
    Point_test
    Random_test
    Ray_test
    Timer_test
    Vector_test
)

include(GoogleTest)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/../bin)
set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)

cmake_policy(PUSH)
set(CMAKE_WARN_DEPRECATED OFF CACHE BOOL "" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../lib/googletest ${CMAKE_CURRENT_BINARY_DIR}/lib/googletest)
cmake_policy(POP)

# Let gtest's own include dirs be treated as system headers
get_target_property(GTEST_INCLUDES gtest INTERFACE_INCLUDE_DIRECTORIES)
set_target_properties(gtest PROPERTIES
    INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${GTEST_INCLUDES}"
)

foreach(TEST ${TESTS})
    add_executable(${TEST} ${TEST}.cpp)

    set_target_properties(${TEST} PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS OFF
    )

    target_link_libraries(${TEST} PRIVATE
        ${PROJECT_NAME}
        gtest_main
        compile_options
        sanitizers
    )

    gtest_discover_tests(${TEST})
    # gtest_add_tests(TARGET ${TEST})
endforeach()
