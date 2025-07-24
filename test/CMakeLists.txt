file(GLOB TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

add_executable(RTB_Tests ${TEST_SOURCES})

target_include_directories(RTB_Tests PRIVATE
    ${CMAKE_SOURCE_DIR}/inc
    ${CMAKE_SOURCE_DIR}/lib
)

include_directories(${CMAKE_SOURCE_DIR}/lib)

target_link_libraries(RTB_Tests
    gtest_main
)

include(GoogleTest)
gtest_discover_tests(RTB_Tests)
