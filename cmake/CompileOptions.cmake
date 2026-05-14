add_library(compile_options INTERFACE)
target_compile_options(compile_options INTERFACE
    $<$<CONFIG:Debug>:
    -O0
    -g
    -Wall
    -Wextra
    -Wpedantic
    -Wshadow
    -Wconversion
    -Wdouble-promotion
    -Wnon-virtual-dtor
    -Wold-style-cast
    -Wcast-align
    -Woverloaded-virtual
    -Wnull-dereference
    -Wmisleading-indentation
    -Wfloat-equal
    -Wswitch-enum
    -Wundef
    -Wformat=2
    -Warray-bounds
    >
    $<$<CONFIG:Release>:
    -O3
    -flto
    -ffast-math
    -DNDEBUG
    -fno-exceptions
    -fno-rtti
    -s
    -fvisibility=hidden
    -fvisibility-inlines-hidden
    -fno-unwind-tables
    -fno-asynchronous-unwind-tables
    >
)

target_link_options(compile_options INTERFACE
    $<$<CONFIG:Release>:
    $<$<NOT:$<PLATFORM_ID:Darwin>>:
    -Wl,--strip-all
    -Wl,--discard-all
    -Wl,--build-id=none
    >
    $<$<PLATFORM_ID:Darwin>:
    -Wl,-dead_strip
    -Wl,-x
    >
    >
)

add_library(sanitizers INTERFACE)
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(sanitizers INTERFACE
        $<$<CONFIG:Debug>:
        -fsanitize=address,undefined,bounds
        -fno-omit-frame-pointer
        >
    )
    target_link_options(sanitizers INTERFACE
        $<$<CONFIG:Debug>:
        -fsanitize=address,undefined,bounds
        >
    )
endif()
