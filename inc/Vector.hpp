#ifndef VECTOR_HPP
#define VECTOR_HPP

// STL
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <type_traits>

// RTB
#include "Point.hpp"
#include "Standards.hpp"

namespace RTB {

// ==============================
// Forward declarations (non-members)
// ==============================

template <typename T, std::size_t N>
class Vector;

template <typename T, typename T2, std::size_t N>
auto CrossProduct(const Vector<T, N>& v1, const Vector<T2, N>& v2)
    -> Vector<typename std::common_type<T, T2>::type, N>;

template <typename T, typename T2, std::size_t N>
auto DotProduct(const Vector<T, N>& v1, const Vector<T2, N>& v2) ->
    typename std::common_type<T, T2>::type;

template <typename T, std::size_t N>
std::ostream& operator<<(std::ostream& out, const Vector<T, N>& v);

template <typename T, std::size_t N>
Vector<T, N> operator+(const T& scalar, const Vector<T, N>& v);

template <typename T, std::size_t N>
Vector<T, N> operator-(const T& scalar, const Vector<T, N>& v);

template <typename T, std::size_t N>
Vector<T, N> operator*(const T& scalar, const Vector<T, N>& v);

template <typename T, std::size_t N>
Vector<T, N> operator-(const Vector<T, N>& v);

template <typename T, std::size_t N>
bool operator==(const Vector<T, N>& lhs, const Vector<T, N>& rhs);

template <typename T, std::size_t N>
bool operator!=(const Vector<T, N>& lhs, const Vector<T, N>& rhs);

// ==============================
// Vector Class
// ==============================

/**
 * @brief Mathematical Vector class.
 *
 * Represents a fixed-size vector of dimension N and data type T.
 *
 * @tparam T Data type of the vector elements (e.g., float, double).
 * @tparam N Size (dimension) of the vector.
 */
template <typename T, std::size_t N>
class Vector {
   public:
    Vector();
    Vector(const std::initializer_list<T>& values);
    Vector(const std::array<T, N>& values);
    Vector(const Point<T, N>& startPoint, const Point<T, N>& endPoint);
    Vector(const Point<T, N>& endPoint);

    constexpr std::size_t GetSize() const;
    std::array<T, N> GetComponents() const;
    void SetComponents(const std::array<T, N>& values);

    T& operator[](std::size_t index);
    const T& operator[](std::size_t index) const;

    // Arithmetic operators
    Vector operator+(const T t) const;
    Vector operator+(const Vector& v) const;
    Vector operator-(const T t) const;
    Vector operator-(const Vector& v) const;
    Vector operator*(const T t) const;
    Vector operator*(const Vector& v) const;
    Vector operator/(const T t) const;

    // Compound assignment
    Vector& operator+=(const T t);
    Vector& operator+=(const Vector& v);
    Vector& operator-=(const T t);
    Vector& operator-=(const Vector& v);
    Vector& operator*=(const T t);
    Vector& operator*=(const Vector& v);
    Vector& operator/=(const T t);

    /**
     * @brief Returns the Euclidean length (magnitude) of the vector.
     */
    T Magnitude() const;

    /**
     * @brief Returns the squared length of the vector.
     */
    T MagnitudeSquared() const;

    /**
     * @brief Prints the vector components to std::cout.
     */
    void Print() const;

    /**
     * @brief Inverts the vector (negates all components).
     */
    Vector& Invert();

    /**
     * @brief Gives a new normalized copy
     */
    Vector Normalize() const;

    /**
     * @brief Normalizes the vector in-place.
     */
    Vector& NormalizeInPlace();

   private:
    std::array<T, N> m_components;
};

// ==============================
// Implementation
// ==============================

template <typename T, std::size_t N>
Vector<T, N>::Vector() : m_components{} {};

template <typename T, std::size_t N>
Vector<T, N>::Vector(const std::initializer_list<T>& values) {
    assert(values.size() == N);
    std::copy(values.begin(), values.end(), m_components.begin());
}

template <typename T, std::size_t N>
Vector<T, N>::Vector(const std::array<T, N>& values) : m_components(values) {}

template <typename T, std::size_t N>
Vector<T, N>::Vector(const Point<T, N>& startPoint,
                     const Point<T, N>& endPoint) {
    for (std::size_t i = 0; i < N; ++i)
        m_components[i] = endPoint[i] - startPoint[i];
}

template <typename T, std::size_t N>
Vector<T, N>::Vector(const Point<T, N>& endPoint) {
    for (std::size_t i = 0; i < N; ++i)
        m_components[i] = endPoint[i];
}

template <typename T, std::size_t N>
constexpr std::size_t Vector<T, N>::GetSize() const {
    return N;
}

template <typename T, std::size_t N>
std::array<T, N> Vector<T, N>::GetComponents() const {
    return m_components;
}

template <typename T, std::size_t N>
void Vector<T, N>::SetComponents(const std::array<T, N>& values) {
    m_components = values;
}

template <typename T, std::size_t N>
T& Vector<T, N>::operator[](std::size_t index) {
    assert(index < N);
    return m_components[index];
}

template <typename T, std::size_t N>
const T& Vector<T, N>::operator[](std::size_t index) const {
    assert(index < N);
    return m_components[index];
}

// Arithmetic operators
template <typename T, std::size_t N>
Vector<T, N> Vector<T, N>::operator+(const T t) const {
    Vector result;
    std::transform(m_components.begin(),
                   m_components.end(),
                   result.m_components.begin(),
                   [t](T c) { return c + t; });
    return result;
}

template <typename T, std::size_t N>
Vector<T, N> Vector<T, N>::operator+(const Vector& v) const {
    Vector result = *this;
    result += v;
    return result;
}

template <typename T, std::size_t N>
Vector<T, N> Vector<T, N>::operator-(const T t) const {
    Vector result;
    std::transform(m_components.begin(),
                   m_components.end(),
                   result.m_components.begin(),
                   [t](T c) { return c - t; });
    return result;
}

template <typename T, std::size_t N>
Vector<T, N> Vector<T, N>::operator-(const Vector& v) const {
    Vector result = *this;
    result -= v;
    return result;
}

template <typename T, std::size_t N>
Vector<T, N> Vector<T, N>::operator*(const T t) const {
    Vector result = *this;
    result *= t;
    return result;
}

template <typename T, std::size_t N>
Vector<T, N> Vector<T, N>::operator*(const Vector& v) const {
    Vector result = *this;
    for (std::size_t i = 0; i < N; ++i)
        result[i] *= v[i];
    return result;
}

template <typename T, std::size_t N>
Vector<T, N> Vector<T, N>::operator/(const T t) const {
    assert(t != static_cast<T>(0));
    Vector result = *this;
    result /= t;
    return result;
}

// Compound assignments
template <typename T, std::size_t N>
Vector<T, N>& Vector<T, N>::operator+=(const T t) {
    std::transform(m_components.begin(),
                   m_components.end(),
                   m_components.begin(),
                   [t](T c) { return c + t; });
    return *this;
}

template <typename T, std::size_t N>
Vector<T, N>& Vector<T, N>::operator+=(const Vector& v) {
    for (std::size_t i = 0; i < N; ++i)
        m_components[i] += v[i];
    return *this;
}

template <typename T, std::size_t N>
Vector<T, N>& Vector<T, N>::operator-=(const T t) {
    std::transform(m_components.begin(),
                   m_components.end(),
                   m_components.begin(),
                   [t](T c) { return c - t; });
    return *this;
}

template <typename T, std::size_t N>
Vector<T, N>& Vector<T, N>::operator-=(const Vector& v) {
    for (std::size_t i = 0; i < N; ++i)
        m_components[i] -= v[i];
    return *this;
}

template <typename T, std::size_t N>
Vector<T, N>& Vector<T, N>::operator*=(const T t) {
    for (auto& comp : m_components)
        comp *= t;
    return *this;
}

template <typename T, std::size_t N>
Vector<T, N>& Vector<T, N>::operator*=(const Vector& v) {
    for (std::size_t i = 0; i < N; ++i)
        m_components[i] *= v[i];
    return *this;
}

template <typename T, std::size_t N>
Vector<T, N>& Vector<T, N>::operator/=(const T t) {
    assert(t != static_cast<T>(0));
    for (auto& comp : m_components)
        comp /= t;
    return *this;
}

template <typename T, std::size_t N>
T Vector<T, N>::MagnitudeSquared() const {
    T sum = static_cast<T>(0);
    for (const auto& c : m_components)
        sum += c * c;
    return sum;
}

template <typename T, std::size_t N>
T Vector<T, N>::Magnitude() const {
    return std::sqrt(MagnitudeSquared());
}

template <typename T, std::size_t N>
void Vector<T, N>::Print() const {
    std::cout << "[Vector] - ";
    std::cout << *this << "\n";
}

template <typename T, std::size_t N>
Vector<T, N> Vector<T, N>::Normalize() const {
    T len = Magnitude();
    if (len != static_cast<T>(0)) {
        return *this / len;
    }
    return *this;
}

template <typename T, std::size_t N>
Vector<T, N>& Vector<T, N>::NormalizeInPlace() {
    T len = Magnitude();
    if (len != static_cast<T>(0)) {
        *this /= len;
    }
    return *this;
}

template <typename T, std::size_t N>
Vector<T, N>& Vector<T, N>::Invert() {
    for (auto& c : m_components)
        c = -c;
    return *this;
}

// ==============================
// Non-member function definitions
// ==============================

template <typename T, typename T2, std::size_t N>
auto CrossProduct(const Vector<T, N>& v1, const Vector<T2, N>& v2)
    -> Vector<typename std::common_type<T, T2>::type, N> {
    static_assert(N == 3, "Cross product is only defined for 3D vectors.");
    using R = typename std::common_type<T, T2>::type;

    return Vector<R, N>{static_cast<R>(v1[1] * v2[2] - v1[2] * v2[1]),
                        static_cast<R>(v1[2] * v2[0] - v1[0] * v2[2]),
                        static_cast<R>(v1[0] * v2[1] - v1[1] * v2[0])};
}

template <typename T, typename T2, std::size_t N>
auto DotProduct(const Vector<T, N>& v1, const Vector<T2, N>& v2) ->
    typename std::common_type<T, T2>::type {
    using CommonT = typename std::common_type<T, T2>::type;
    CommonT result = static_cast<CommonT>(0);
    for (std::size_t i = 0; i < N; ++i) {
        result += static_cast<CommonT>(v1[i] * v2[i]);
    }
    return result;
}

template <typename T, std::size_t N>
std::ostream& operator<<(std::ostream& out, const Vector<T, N>& v) {
    out << "(";
    for (std::size_t i = 0; i < N; ++i) {
        out << v[i];
        if (i < N - 1)
            out << ", ";
    }
    out << ")";
    return out;
}

template <typename T, std::size_t N>
Vector<T, N> operator+(const T& scalar, const Vector<T, N>& v) {
    return v + scalar;
}

template <typename T, std::size_t N>
Vector<T, N> operator-(const T& scalar, const Vector<T, N>& v) {
    Vector<T, N> result;
    for (std::size_t i = 0; i < N; ++i)
        result[i] = scalar - v[i];
    return result;
}

template <typename T, std::size_t N>
Vector<T, N> operator*(const T& scalar, const Vector<T, N>& v) {
    return v * scalar;
}

template <typename T, std::size_t N>
Vector<T, N> operator-(const Vector<T, N>& v) {
    Vector<T, N> result;
    for (std::size_t i = 0; i < N; ++i)
        result[i] = -v[i];
    return result;
}

template <typename T, std::size_t N>
bool operator==(const Vector<T, N>& lhs, const Vector<T, N>& rhs) {
    return lhs.GetComponents() == rhs.GetComponents();
}

template <typename T, std::size_t N>
bool operator!=(const Vector<T, N>& lhs, const Vector<T, N>& rhs) {
    return !(lhs == rhs);
}

// ==============================
// Explicit instantiations
// ==============================

template class Vector<float, 2>;
using Vec2f = Vector<float, 2>;

template class Vector<float, 3>;
using Vec3f = Vector<float, 3>;

template class Vector<float, 4>;
using Vec4f = Vector<float, 4>;

}  // namespace RTB

#endif /* VECTOR_HPP */
