add_executable(DEV Dev.cpp)

set_target_properties(DEV PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

target_link_libraries(DEV PRIVATE
    ${PROJECT_NAME}
    compile_options
    sanitizers
)
