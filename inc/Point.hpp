#ifndef POINT_HPP
#define POINT_HPP

// STL
#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <stdexcept>
#include <type_traits>

// RTB
#include "Standards.hpp"

namespace RTB {

// Interface
template <typename T, std::size_t N>
class Point {
   public:
    Point();

    Point(const std::initializer_list<T> &values);

    std::array<T, N> GetCoords() const;

    void SetCoords(const std::array<T, N> &newCoords);

    T &operator[](std::size_t index);
    const T &operator[](std::size_t index) const;

    void Print() const;

   private:
    std::array<T, N> m_coords;

};  // Point

// Implementation
template <typename T, std::size_t N>
Point<T, N>::Point() {
    m_coords.fill(static_cast<T>(0));
}

template <typename T, std::size_t N>
Point<T, N>::Point(const std::initializer_list<T> &values) {
    if (values.size() != N) {
        throw std::invalid_argument(
            "Point constructor : Initializer list size does not match "
            "point dimension.");
    }
    std::copy(values.begin(), values.end(), m_coords.begin());
}
template <typename T, std::size_t N>
std::array<T, N> Point<T, N>::GetCoords() const {
    return m_coords;
}

template <typename T, std::size_t N>
void Point<T, N>::SetCoords(const std::array<T, N> &newCoords) {
    m_coords = newCoords;
}

template <typename T, std::size_t N>
T &Point<T, N>::operator[](std::size_t index) {
    if (index >= N) {
        throw std::out_of_range("Index out of range");
    }
    return m_coords[index];
}

template <typename T, std::size_t N>
const T &Point<T, N>::operator[](std::size_t index) const {
    if (index >= N) {
        throw std::out_of_range("Index out of range");
    }
    return m_coords[index];
}

template <typename T, std::size_t N>
void Point<T, N>::Print() const {
    std::cout << "(";
    for (std::size_t i = 0; i < N; i++) {
        std::cout << m_coords[i];
        if (i < N - 1) {
            std::cout << ", ";
        }
    }
    std::cout << ")" << std::endl;
}

// Template non-member functions
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

// explicit instantiation
template class Point<RESOLUTION, 3>;
template class Point<float,3>;
using Point3R = Point<RESOLUTION, 3>;

}  // namespace RTB

#endif  // POINT_HPP