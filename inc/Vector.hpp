#ifndef VECTOR_HPP
#define VECTOR_HPP

#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <stdexcept>
#include <type_traits>

#include "Point.hpp"
#include "Standards.hpp"

namespace RTB {

template <typename T, std::size_t N>
class Vector {
    static_assert(std::is_arithmetic<T>::value,
                  "Vector can only use arithmetic types");

    static_assert(N > 0, "Vector dimension must be greater than zero");

   private:
    std::array<T, N> components;

   public:
    Vector() {
        components.fill(static_cast<T>(0));
    }

    Vector(const std::initializer_list<T> &values) {
        if (values.size() != N) {
            throw std::invalid_argument(
                "Initializer list size does not match vector dimension.");
        }
        std::copy(values.begin(), values.end(), components.begin());
    }

    Vector(const Point3R &startPoint, const Point3R &endPoint) {
        components[0] = endPoint[0] - startPoint[0];  // X component
        components[1] = endPoint[1] - startPoint[1];  // Y component
        components[2] = endPoint[2] - startPoint[2];  // Z component
    }

    ~Vector() = default;

    std::size_t Size() {
        return N;
    }

    std::array<T, N> GetComponents() {
        return components;
    }

    void SetComponents(const std::array<T, N> &values) {
        components = values;
    }

    // Only for 1D, 2D, 3D
    T x() const {
        static_assert(N < 4,
                      "This operation can only be performed on 3D vectors");
        return components[0];
    }
    T y() const {
        static_assert(N < 4,
                      "This operation can only be performed on 3D vectors");
        return components[1];
    }
    T z() const {
        static_assert(N < 4,
                      "This operation can only be performed on 3D vectors");
        return components[2];
    }

    T &operator[](std::size_t index) {
        if (index >= N) {
            throw std::out_of_range("Index out of range");
        }
        return components[index];
    }

    const T &operator[](std::size_t index) const {
        if (index >= N) {
            throw std::out_of_range("Index out of range");
        }
        return components[index];
    }

    // Add a vector
    Vector operator+(const Vector &v) const {
        Vector result = *this;
        result += v;
        return result;
    }

    // Negation
    Vector operator-() const {
        Vector result;

        std::transform(components.begin(),
                       components.end(),
                       result.components.begin(),
                       [](T x) { return -x; });
        return result;
    }

    // Subtract a vector
    Vector operator-(const Vector &v) const {
        Vector result = *this;
        result -= v;
        return result;
    }

    // Multiply by constant
    Vector operator*(const T t) const {
        Vector result = *this;
        result *= t;
        return result;
    }

    // Multiply by vector
    Vector operator*(const Vector &v) const {
        Vector result = *this;
        for (std::size_t i = 0; i < N; ++i) {
            result[i] *= v[i];  // divide each component by the scalar t
        }
        return result;
    }

    // Divide by constant
    Vector operator/(const T t) const {
        if (t == static_cast<T>(0)) {
            throw std::invalid_argument("Division by zero is not allowed.");
        }
        Vector result = *this;
        // for (std::size_t i = 0; i < N; ++i) {
        //     result[i] /= t;  // divide each component by the scalar t
        // }
        result /= t;
        return result;
    }

    Vector &operator+=(const Vector &v) {
        for (std::size_t i = 0; i < N; ++i) {
            components[i] += v[i];
        }
        return *this;
    }

    Vector &operator-=(const Vector &v) {
        for (std::size_t i = 0; i < N; ++i) {
            components[i] -= v[i];
        }
        return *this;
    }

    Vector &operator*=(const T t) {
        for (std::size_t i = 0; i < N; ++i) {
            components[i] *= t;
        }
        return *this;
    }

    Vector &operator/=(const T t) {
        return *this *= static_cast<T>(1) / t;
    }

    T length() const {
        return std::sqrt(length_squared());
    }

    T length_squared() const {
        T sum = static_cast<T>(0);
        for (std::size_t i = 0; i < N; ++i) {
            sum += components[i] * components[i];
        }
        return sum;
    }

    void print() const {
        std::cout << "(";
        for (std::size_t i = 0; i < N; i++) {
            std::cout << components[i];
            if (i < N - 1) {
                std::cout << ", ";
            }
        }
        std::cout << ")" << "\n";
    }

};  // Vector

// Template functions

template <typename T, std::size_t N>
Vector<T, N> unit_vector(const Vector<T, N> &v) {
    return v / static_cast<T>(v.length());
}

template <typename T, typename T2, std::size_t N>
auto crossprod(const Vector<T, N> &v1, const Vector<T2, N> &v2)
    -> Vector<typename std::common_type<T, T2>::type, N> {
    static_assert(N == 3, "Cross product is only defined for 3D vectors.");
    using ResultType = typename std::common_type<T, T2>::type;

    return Vector<ResultType, N>{
        static_cast<ResultType>(v1[1] * v2[2] - v1[2] * v2[1]),
        static_cast<ResultType>(v1[2] * v2[0] - v1[0] * v2[2]),
        static_cast<ResultType>(v1[0] * v2[1] - v1[1] * v2[0])};
}

template <typename T, typename T2, std::size_t N>
auto dotprod(const Vector<T, N> &v1, const Vector<T2, N> &v2) ->
    typename std::common_type<T, T2>::type {
    static_assert(N == 3, "Dot product is only defined for 3D vectors.");
    using ResultType = typename std::common_type<T, T2>::type;

    return static_cast<ResultType>(v1[0] * v2[0] + v1[1] * v2[1] +
                                   v1[2] * v2[2]);
}

template <typename T, std::size_t N>
inline std::ostream &operator<<(std::ostream &out, const Vector<T, N> &v) {
    // for loop over each element
    return out << v[0] << ' ' << v[1] << ' ' << v[2];
}

// using Vec3f = Vector<float, 3>;
using Vec3R = Vector<RESOLUTION, 3>;

}  // namespace RTB
#endif  // VECTOR_HPP