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
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <type_traits>

// a colour is represented by an array of 3 or 4 bytes: RGB(A)

namespace RTB {

template <typename T, std::size_t N>
class Colour {
    static_assert(std::is_arithmetic_v<T>,
                  "Colour<T, N>: T must be an arithmetic type.");
    static_assert(N > 0, "Colour<T, N>: N must be greater than zero.");

   public:
    Colour() {
        m_components.fill(static_cast<T>(0));
    }

    /**
     * @brief Constructs from an initializer list.
     *
     * If values.size() != N, logs a mismatch (Debug builds only) and
     * copies only the overlapping elements: extra values are ignored,
     * and any remaining components stay zero-initialized.
     */
    Colour(const std::initializer_list<T>& values) : m_components{} {
        if (values.size() != N) {
            RTB_DEBUG_LOG(
                "[COLOUR_HPP] @ L22: Initializer list size does not match "
                "dimension.\n");
        }
        const auto count = std::min(values.size(), N);
        std::copy_n(values.begin(), count, m_components.begin());
    }

    void clamp(const T& lowerbound, const T& upperbound) {
        for (auto& comp : m_components) {
            if (comp < lowerbound) {
                comp = lowerbound;
            } else if (comp > upperbound) {
                comp = upperbound;
            }
        }
    }

    [[nodiscard]] const std::array<T, N>& getComponents() const {
        return m_components;
    }

    // Overloaded functions
    //  Multiply (constant or other color)
    //  Add (constant or other color)
   private:
    std::array<T, N> m_components;
};

using Colour8_3 = Colour<uint8_t, 3>;  // RGB - 8 bits per channel, 0-255
using Colour8_4 = Colour<uint8_t, 4>;  // RGBA - 8 bits per channel, 0-255

}  // namespace RTB
#endif /* RTB_COLOUR_HPP */
