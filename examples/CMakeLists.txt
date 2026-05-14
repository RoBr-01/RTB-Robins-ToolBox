set(EXAMPLES

)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/../bin)

foreach(EXAMPLE ${EXAMPLES})
    add_executable(${EXAMPLE} ${EXAMPLE}.cpp)
    target_link_libraries(${EXAMPLE} PRIVATE
        ${PROJECT_NAME}
        compile_options
        sanitizers
    )
    # target_include_directories(${EXAMPLE} PRIVATE
    # )
endforeach()


