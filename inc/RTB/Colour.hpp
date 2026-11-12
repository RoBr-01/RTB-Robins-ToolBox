/*
 ____ _____ ____
|  _ \_   _| __ )
| |_) || | |  _ \
|  _ < | | | |_) |
|_| \_\|_| |____/

 Colour.hpp
 Fixed-size RGB/RGBA colour type.

 Copyright (c) 2026 Robin Breugelmans
 Licensed under the MIT License. See LICENSE in the project root.
*/

#ifndef RTB_COLOUR_HPP
#define RTB_COLOUR_HPP

#include <RTB/Debug.hpp>
#include <array>
#include <cstddef>
#include <cstdint>
#include <initializer_list>

// a colour is represented by an array of 3 or 4 bytes: RGB(A)

namespace RTB {

template <typename T, std::size_t N>
class Colour {
   public:
    Colour() {
        m_components.fill(static_cast<T>(0));
    }

    Colour(const std::initializer_list<T>& values) {
        if (values.size() != N) {
            RTB_DEBUG_LOG(
                "[COLOUR_HPP] @ L22: Initializer list size does not match "
                "dimension.\n");
        }
        std::copy(values.begin(), values.end(), m_components.begin());
    }

    void update() {}
    void clamp(const T& lowerbound, const T& upperbound) {
        for (auto& comp : m_components) {
            if (comp < lowerbound) {
                comp = lowerbound;
            } else if (comp > upperbound) {
                comp = upperbound;
            }
        }
    }

    // Overloaded functions
    //  Multiply (constant or other color)
    //  Add (constant or other color)
   private:
    std::array<T, N> m_components;
};

// Using int_fast8_t because its not worth risking wanting exactly 8 bits when
// it might negatively impact speed
using Colour8_3 = Colour<int_fast8_t, 3>;  // RGB - 8 bits per channel
using Colour8_4 = Colour<int_fast8_t, 4>;  // RGBA - 8 bits per channel

}  // namespace RTB
#endif /* RTB_COLOUR_HPP */
