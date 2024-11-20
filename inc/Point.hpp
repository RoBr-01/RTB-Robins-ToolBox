#ifndef POINT_HPP
#define POINT_HPP

#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <stdexcept>
#include <type_traits>

#include "Standards.hpp"

namespace RTB {

template <typename T, std::size_t N>
class Point {
  static_assert(std::is_arithmetic<T>::value,
                "Point can only use arithmetic types");

  static_assert(N > 0, "Point dimension must be greater than zero");

 private:
  std::array<T, N> coords;

 public:
  Point() { coords.fill(static_cast<T>(0)); }

  Point(const std::initializer_list<T> &values) {
    if (values.size() != N) {
      throw std::invalid_argument(
          "Initializer list size does not match point dimension.");
    }
    std::copy(values.begin(), values.end(), coords.begin());
  }

  T &operator[](std::size_t index) {
    if (index >= N) {
      throw std::out_of_range("Index out of range");
    }
    return coords[index];
  }

  const T &operator[](std::size_t index) const {
    if (index >= N) {
      throw std::out_of_range("Index out of range");
    }
    return coords[index];
  }

  void print() const {
    std::cout << "(";
    for (std::size_t i = 0; i < N; i++) {
      std::cout << coords[i];
      if (i < N - 1) {
        std::cout << ", ";
      }
    }
    std::cout << ")" << std::endl;
  }

};  // Point

template <typename T1, typename T2, std::size_t N>
RESOLUTION distance_2points(const Point<T1, N> &P1, const Point<T2, N> &P2) {
  RESOLUTION sum = 0.0;
  for (std::size_t i = 0; i < N; i++) {
    RESOLUTION diff = P2[i] - P1[i];
    sum += diff * diff;
  }
  return std::sqrt(static_cast<RESOLUTION>(sum));
}

template <typename T1, typename T2, std::size_t N>
Point<RESOLUTION, N> midpoint(const Point<T1, N> &P1, const Point<T2, N> &P2) {
  Point<RESOLUTION, N> mid;
  for (std::size_t i = 0; i < N; i++) {
    mid[i] = (P1[i] + P2[i]) / 2.0;
  }

  return mid;
}

using Point3f = Point<float, 3>;

}  // namespace RTB

#endif  // POINT_HPP