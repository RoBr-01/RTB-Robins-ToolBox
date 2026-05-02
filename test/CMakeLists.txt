add_executable(${TEST_NAME})

target_sources(${TEST_NAME} PRIVATE 
    Ellipsoid_test.cpp
    Timer_test.cpp
    Vector_test.cpp
)

# This allows us to call add_subdirecoty while googletest is actually not a subdirectory
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../lib/googletest ${CMAKE_CURRENT_BINARY_DIR}/lib/googletest)
target_include_directories(${TEST_NAME} INTERFACE ${CMAKE_SOURCE_DIR}/lib/googletest/googletest/include)
target_link_libraries(${TEST_NAME} PRIVATE RTB GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(${TEST_NAME})