/*
 ____ _____ ____
|  _ \_   _| __ )
| |_) || | |  _ \
|  _ < | | | |_) |
|_| \_\|_| |____/

 Debug.hpp
 Debug-only logging macro, compiled out entirely in Release builds.

 Copyright (c) 2026 Robin Breugelmans
 Licensed under the MIT License. See LICENSE in the project root.
*/

#ifndef RTB_DEBUG_HPP
#define RTB_DEBUG_HPP

#include <iostream>

/**
 * @brief Streams to std::cerr in debug builds, compiles to nothing when
 *        NDEBUG is defined (e.g. CMake Release configs).
 *
 * Example:
 *   RTB_DEBUG_LOG("No valid intersection, value was " << x << "\n");
 */
#ifdef NDEBUG
#define RTB_DEBUG_LOG(...) \
    do {                   \
    } while (0)
#else
#define RTB_DEBUG_LOG(...)        \
    do {                          \
        std::cerr << __VA_ARGS__; \
    } while (0)
#endif

#endif  // RTB_DEBUG_HPP
