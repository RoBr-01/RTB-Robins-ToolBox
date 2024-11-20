#ifndef COLOUR_HPP
#define COLOUR_HPP

#include <array>
#include <cstddef>
// a colour is represented by an array of 3 or 4 bytes: RGB(A)
// Using int_fast8_t because its not worth risking wanting exactly 8 bits when
// it might negatively impact speed

namespace RTB {

class Colour {
 private:
  std::array<int_fast8_t, 3> components;

 public:
  Colour() : components{0, 0, 0} {}

  ~Colour();

  Colour(int_fast8_t c1 = 0, int_fast8_t c2 = 0, int_fast8_t c3 = 0)
      : components{c1, c2, c3} {}
};

#endif  // COLOUR_HPP
}  // RTB