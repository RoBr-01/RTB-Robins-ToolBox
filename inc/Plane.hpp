#ifndef PLANE_HPP
#define PLANE_HPP

#include <array>

#include "Point.hpp"
#include "Standards.hpp"
#include "Vector.hpp"

namespace RTB {

template <typename T>
class Plane {
 public:
  Plane() : m_coefficients{0, 0, 0, 0} {}
  Plane(T a, T b, T c, T d) : m_coefficients{a, b, c, d} {};

  Plane(Point3R A, Point3R B, Point3R C) {
    m_coefficients[0] =
        (B[1] - A[1]) * (C[2] - A[2]) - (C[1] - A[1]) * (B[2] - A[2]);
    m_coefficients[1] =
        (B[2] - A[2]) * (C[0] - A[0]) - (C[2] - A[2]) * (B[0] - A[0]);
    m_coefficients[2] =
        (B[0] - A[0]) * (C[1] - A[1]) - (C[0] - A[0]) * (B[1] - A[1]);
    m_coefficients[3] = -(m_coefficients[0] * A[0] + m_coefficients[1] * A[1] +
                          m_coefficients[2] * A[2]);
  }

  void Invert() {
    m_coefficients[0] = -m_coefficients[0];
    m_coefficients[1] = -m_coefficients[1];
    m_coefficients[2] = -m_coefficients[2];
    m_coefficients[3] = -m_coefficients[3];
  }

  void Normalize() { m_coefficients = unit_vector(m_coefficients); }

  void Print() const {
    std::cout << "Plane coefficients: ";
    m_coefficients.print();  // Call the print method of the Vector class
  }

 private:
  //   std::array<T, 4> m_coefficients;
  Vector<T, 4> m_coefficients;  // Because member functions are hidden anyway
};

using PlaneR = Plane<RESOLUTION>;
}  // namespace RTB

#endif  // PLANE_HPP