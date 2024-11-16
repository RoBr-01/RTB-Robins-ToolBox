#ifndef POINT_HPP
#define POINT_HPP

#include <cstddef>
#include <array>
#include <cmath>
#include <stdexcept>
#include <type_traits>

namespace RTB{

template <typename T, std::size_t N>
class Point {

    static_assert(std::is_arithmetic<T>::value, "Point can only use arithmetic types");

private:

    std::array<T, N> coords; // Array to hold coordinates

public:

    Point() {
        coords.fill(static_cast<T>(0));
    }

    Point(const std::initializer_list<T>& values) {
        if (values.size() != N) {
            throw std::invalid_argument("Initializer list size does not match point dimension.");
        }
        std::copy(values.begin(), values.end(), coords.begin());
    }

    T& operator[](std::size_t index) {
        if (index >= N) {
            throw std::out_of_range("Index out of range");
        }
        return coords[index];
    }

    const T& operator[](std::size_t index) const {
        if (index >= N) {
            throw std::out_of_range("Index out of range");
        }
        return coords[index];
    }

    double distanceTo(const Point<T, N>& other) const {
        T sum = static_cast<T>(0);
        for (std::size_t i = 0; i < N; ++i) {
            T diff = coords[i] - other.coords[i];
            sum += diff * diff;
        }
        return std::sqrt(static_cast<double>(sum));
    }

    void print() const {
        std::cout << "(";
        for (std::size_t i = 0; i < N; ++i) {
            std::cout << coords[i];
            if (i < N - 1) {
                std::cout << ", ";
            }
        }
        std::cout << ")" << std::endl;
    }
};


}
#endif