#ifndef VECTOR_HPP
#define VECTOR_HPP

#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <stdexcept>
#include <type_traits>
#include "Standards.hpp"

namespace RTB{

template <typename T, std::size_t N>
class Vector{

    static_assert(std::is_arithmetic<T>::value, 
    "Vector can only use arithmetic types");

    static_assert(N > 0, "Vector dimension must be greater than zero");

private:

    std::array<T, N> components;

public:

    Vector() {
        components.fill(static_cast<T>(0));
    }

    Vector(const std::initializer_list<T>& values) {
        if (values.size() != N) {
            throw std::invalid_argument(
                "Initializer list size does not match vector dimension.");
        }
        std::copy(values.begin(), values.end(), components.begin());
    }

    ~Vector() = default;

    RESOLUTION x() const { return components[0]; }
    RESOLUTION y() const { return components[1]; }
    RESOLUTION z() const { return components[2]; }

    T& operator[](std::size_t index) {
        if (index >= N) {
            throw std::out_of_range("Index out of range");
        }
        return components[index];
    }

    const T& operator[](std::size_t index) const {
        if (index >= N) {
            throw std::out_of_range("Index out of range");
        }
        return components[index];
    }

    Vector operator+(const Vector& v) const {
        Vector result = *this;
        result += v;
        return result;
    }

    Vector operator-(const Vector& v) const {
        Vector result = *this;
        result -= v;
        return result;
    }

    Vector operator*(T t) const {
        Vector result = *this;
        result *= t;
        return result;
    }

    Vector operator/(T t) const {
        if (t == static_cast<T>(0)) {
            throw std::invalid_argument("Division by zero is not allowed.");
        }
        Vector result = *this;
        for (std::size_t i = 0; i < N; ++i) {
            result[i] /= t;  // divide each component by the scalar t
        }
        return result;
    }

    Vector& operator+=(const Vector& v) {
        for (std::size_t i = 0; i < N; ++i) {
            components[i] += v[i];
        }
        return *this;
    }

    Vector& operator*=(T t) {
        for (std::size_t i = 0; i < N; ++i) {
            components[i] *= t;
        }
        return *this;
    }

    Vector& operator/=(T t) {
        return *this *= static_cast<T>(1) / t;
    }

    RESOLUTION length() const {
        return std::sqrt(length_squared());
    }

    RESOLUTION length_squared() const {
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
        std::cout << ")" << std::endl;
    }

    Vector cross(const Vector& v) const {
        static_assert(N == 3, "Cross product is only defined for 3D vectors.");
        return Vector{
            components[1] * v[2] - components[2] * v[1],
            components[2] * v[0] - components[0] * v[2],
            components[0] * v[1] - components[1] * v[0]
        };
    }

};//Vector

    template <typename T, std::size_t N>
    Vector<T,N> unit_vector(const Vector<T,N>& v) {

        return v/static_cast<T>(v.length());

    }

    using Vec3f = Vector<float,3>;
} //RTB
#endif //VECTOR_HPP