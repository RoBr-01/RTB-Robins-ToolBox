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
    // Default constructor
    Point() {
        coords.fill(static_cast<T>(0)); // Initialize all coordinates to 0
    }

    // Constructor with initializer list
    Point(const std::initializer_list<T>& values) {
        if (values.size() != N) {
            throw std::invalid_argument("Initializer list size does not match point dimension.");
        }
        std::copy(values.begin(), values.end(), coords.begin());
    }

    // Access elements
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

    // Euclidean distance to another point
    double distanceTo(const Point<T, N>& other) const {
        T sum = static_cast<T>(0);
        for (std::size_t i = 0; i < N; ++i) {
            T diff = coords[i] - other.coords[i];
            sum += diff * diff;
        }
        return std::sqrt(static_cast<double>(sum));
    }

    // Print the point
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