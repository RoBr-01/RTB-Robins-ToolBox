#ifndef COLOUR_HPP
#define COLOUR_HPP

#include <array>
#include <cstddef>
// a colour is represented by an array of 3 or 4 bytes: RGB(A)

namespace RTB {

template <typename T, std::size_t N>
class Colour {
   private:
    std::array<T, N> components;

   public:
    Colour() {
        components.fill(static_cast<T>(0));
    }

    Colour(const std::initializer_list<T> &values) {
        if (values.size() != N) {
            throw std::invalid_argument(
                "[COLOUR_HPP] @ L22: Initializer list size does not match "
                "dimension.");
        }
        std::copy(values.begin(), values.end(), components.begin());
    }

    void Update(){}
    void Clamp(const T &lowerbound, const T &upperbound){}

    //Overloaded functions
    // Multiply (constant or other color)
    // Add (constant or other color)

};

// Using int_fast8_t because its not worth risking wanting exactly 8 bits when
// it might negatively impact speed
using Colour8_3 = Colour<int_fast8_t, 3>;  // RGB - 8 bits per channel
using Colour8_4 = Colour<int_fast8_t, 4>;  // RGBA - 8 bits per channel

}  // namespace RTB
#endif  // COLOUR_HPP