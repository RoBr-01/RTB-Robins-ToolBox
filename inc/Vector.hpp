#ifndef VECTOR_HPP
#define VECTOR_HPP

// STL
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <stdexcept>
#include <type_traits>

// RTB
#include "Point.hpp"
#include "Standards.hpp"

namespace RTB {

// Interface
template <typename T, std::size_t N>
class Vector {
   public:
    Vector();

    Vector(const std::initializer_list<T>& values);

    Vector(const Point<T, N>& startPoint, const Point<T, N>& endPoint);

    std::size_t GetSize() const;

    std::array<T, N> GetComponents() const;

    void SetComponents(const std::array<T, N>& values);

    T& operator[](std::size_t index);
    const T& operator[](std::size_t index) const;

    Vector operator+(const T t) const;
    Vector operator+(const Vector& v) const;

    Vector operator-(const T t) const;
    Vector operator-(const Vector& v) const;

    Vector operator*(const T t) const;
    Vector operator*(const Vector& v) const;

    Vector operator/(const T t) const;

    Vector& operator+=(const T t);
    Vector& operator+=(const Vector& v);

    Vector& operator-=(const T t);
    Vector& operator-=(const Vector& v);

    Vector& operator*=(const T t);
    Vector& operator*=(const Vector& v);

    Vector& operator/=(const T t);

    T Length() const;
    T Length_squared() const;

    void Print() const;

   private:
    std::array<T, N> m_components;
};

// Implementation

// Default constructor
template <typename T, std::size_t N>
Vector<T, N>::Vector() {
    m_components.fill(static_cast<T>(0));
}

// Initializer list constructor
template <typename T, std::size_t N>
Vector<T, N>::Vector(const std::initializer_list<T>& values) {
    if (values.size() != N) {
        throw std::invalid_argument(
            "Initializer list size does not match vector dimension.");
    }
    std::copy(values.begin(), values.end(), m_components.begin());
}

// Constructor using two points
template <typename T, std::size_t N>
Vector<T, N>::Vector(const Point<T, N>& startPoint,
                     const Point<T, N>& endPoint) {
    for (std::size_t i = 0; i < N; ++i) {
        m_components[i] = endPoint[i] - startPoint[i];
    }
}

// Get size of vector
template <typename T, std::size_t N>
std::size_t Vector<T, N>::GetSize() const {
    return N;
}

// Get components of vector
template <typename T, std::size_t N>
std::array<T, N> Vector<T, N>::GetComponents() const {
    return m_components;
}

// Set components of vector
template <typename T, std::size_t N>
void Vector<T, N>::SetComponents(const std::array<T, N>& values) {
    m_components = values;
}

// Access operator (mutable)
template <typename T, std::size_t N>
T& Vector<T, N>::operator[](std::size_t index) {
    if (index >= N) {
        throw std::out_of_range("Index out of range");
    }
    return m_components[index];
}

// Access operator (immutable)
template <typename T, std::size_t N>
const T& Vector<T, N>::operator[](std::size_t index) const {
    if (index >= N) {
        throw std::out_of_range("Index out of range");
    }
    return m_components[index];
}

// Scalar addition
template <typename T, std::size_t N>
Vector<T, N> Vector<T, N>::operator+(const T t) const {
    Vector result = *this;
    for (auto& comp : result.m_components) {
        comp += t;
    }
    return result;
}

// Vector addition
template <typename T, std::size_t N>
Vector<T, N> Vector<T, N>::operator+(const Vector& v) const {
    Vector result = *this;
    result += v;
    return result;
}

// Scalar subtraction
template <typename T, std::size_t N>
Vector<T, N> Vector<T, N>::operator-(const T t) const {
    Vector result = *this;
    for (auto& comp : result.m_components) {
        comp -= t;
    }
    return result;
}

// Vector subtraction
template <typename T, std::size_t N>
Vector<T, N> Vector<T, N>::operator-(const Vector& v) const {
    Vector result = *this;
    result -= v;
    return result;
}

// Scalar multiplication
template <typename T, std::size_t N>
Vector<T, N> Vector<T, N>::operator*(const T t) const {
    Vector result = *this;
    result *= t;
    return result;
}

// Element-wise vector multiplication
template <typename T, std::size_t N>
Vector<T, N> Vector<T, N>::operator*(const Vector& v) const {
    Vector result = *this;
    for (std::size_t i = 0; i < N; ++i) {
        result[i] *= v[i];
    }
    return result;
}

// Scalar division
template <typename T, std::size_t N>
Vector<T, N> Vector<T, N>::operator/(const T t) const {
    if (t == static_cast<T>(0)) {
        throw std::invalid_argument("Division by zero is not allowed.");
    }
    Vector result = *this;
    result /= t;
    return result;
}

// Scalar addition assignment
template <typename T, std::size_t N>
Vector<T, N>& Vector<T, N>::operator+=(const T t) {
    for (auto& comp : m_components) {
        comp += t;
    }
    return *this;
}

// Vector addition assignment
template <typename T, std::size_t N>
Vector<T, N>& Vector<T, N>::operator+=(const Vector& v) {
    for (std::size_t i = 0; i < N; ++i) {
        m_components[i] += v[i];
    }
    return *this;
}

// Scalar subtraction assignment
template <typename T, std::size_t N>
Vector<T, N>& Vector<T, N>::operator-=(const T t) {
    for (auto& comp : m_components) {
        comp -= t;
    }
    return *this;
}

// Vector subtraction assignment
template <typename T, std::size_t N>
Vector<T, N>& Vector<T, N>::operator-=(const Vector& v) {
    for (std::size_t i = 0; i < N; ++i) {
        m_components[i] -= v[i];
    }
    return *this;
}

// Scalar multiplication assignment
template <typename T, std::size_t N>
Vector<T, N>& Vector<T, N>::operator*=(const T t) {
    for (auto& comp : m_components) {
        comp *= t;
    }
    return *this;
}

// Element-wise vector multiplication assignment
template <typename T, std::size_t N>
Vector<T, N>& Vector<T, N>::operator*=(const Vector& v) {
    for (std::size_t i = 0; i < N; ++i) {
        m_components[i] *= v[i];
    }
    return *this;
}

// Scalar division assignment
template <typename T, std::size_t N>
Vector<T, N>& Vector<T, N>::operator/=(const T t) {
    if (t == static_cast<T>(0)) {
        throw std::invalid_argument("Division by zero is not allowed.");
    }
    for (auto& comp : m_components) {
        comp /= t;
    }
    return *this;
}

// Length of vector
template <typename T, std::size_t N>
T Vector<T, N>::Length() const {
    return std::sqrt(Length_squared());
}

// Squared length of vector
template <typename T, std::size_t N>
T Vector<T, N>::Length_squared() const {
    T sum = static_cast<T>(0);
    for (const auto& component : m_components) {
        sum += component * component;
    }
    return sum;
}

// Print vector components
template <typename T, std::size_t N>
void Vector<T, N>::Print() const {
    std::cout << "(";
    for (std::size_t i = 0; i < N; i++) {
        std::cout << m_components[i];
        if (i < N - 1) {
            std::cout << ", ";
        }
    }
    std::cout << ")" << "\n";
}

// Template non-member functions
template <typename T, std::size_t N>
Vector<T, N> unit_vector(const Vector<T, N>& v) {
    return v / static_cast<T>(v.Length());
}

template <typename T, typename T2, std::size_t N>
auto crossprod(const Vector<T, N>& v1, const Vector<T2, N>& v2)
    -> Vector<typename std::common_type<T, T2>::type, N> {
    static_assert(N == 3, "Cross product is only defined for 3D vectors.");
    using ResultType = typename std::common_type<T, T2>::type;

    return Vector<ResultType, N>{
        static_cast<ResultType>(v1[1] * v2[2] - v1[2] * v2[1]),
        static_cast<ResultType>(v1[2] * v2[0] - v1[0] * v2[2]),
        static_cast<ResultType>(v1[0] * v2[1] - v1[1] * v2[0])};
}

template <typename T, typename T2, std::size_t N>
auto dotprod(const Vector<T, N>& v1, const Vector<T2, N>& v2) ->
    typename std::common_type<T, T2>::type {
    static_assert(N == 3, "Dot product is only defined for 3D vectors.");
    using ResultType = typename std::common_type<T, T2>::type;

    return static_cast<ResultType>(v1[0] * v2[0] + v1[1] * v2[1] +
                                   v1[2] * v2[2]);
}

template <typename T, std::size_t N>
inline std::ostream& operator<<(std::ostream& out, const Vector<T, N>& v) {
    out << "(";
    for (std::size_t i = 0; i < N; ++i) {
        out << v[i];
        if (i < N - 1) {
            out << ", ";
        }
    }
    out << ")";
    return out;
}

// Explicit instantiation
template class Vector<RESOLUTION, 3>;
using Vec3R = Vector<RESOLUTION, 3>;

}  // namespace RTB

#endif  // VECTOR_HPP