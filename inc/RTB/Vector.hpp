#ifndef VECTOR_HPP
#define VECTOR_HPP

// STL
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <numeric>
#include <type_traits>

// LOCAL
#include <RTB/Point.hpp>

namespace RTB {

// ==============================
// Forward declarations
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
 * Represents a fixed-size mathematical vector of dimension N and scalar
 * type T. T must be an arithmetic type (int, float, double, etc.).
 *
 * @tparam T Scalar type of the vector elements.
 * @tparam N Dimension of the vector.
 */
template <typename T, std::size_t N>
class Vector {
    static_assert(std::is_arithmetic<T>::value,
                  "Vector<T, N>: T must be an arithmetic type.");
    static_assert(N > 0, "Vector<T, N>: N must be greater than zero.");

   public:
    // ---- Construction -------------------------------------------------------

    Vector() : m_components{} {}

    Vector(const std::initializer_list<T>& values) {
        assert(values.size() == N);
        std::copy(values.begin(), values.end(), m_components.begin());
    }

    explicit Vector(const std::array<T, N>& values) : m_components(values) {}

    Vector(const Point<T, N>& startPoint, const Point<T, N>& endPoint) {
        for (std::size_t i = 0; i < N; ++i)
            m_components[i] = endPoint[i] - startPoint[i];
    }

    explicit Vector(const Point<T, N>& endPoint) {
        for (std::size_t i = 0; i < N; ++i)
            m_components[i] = endPoint[i];
    }

    // ---- Accessors ----------------------------------------------------------

    static constexpr std::size_t Size() {
        return N;
    }

    [[nodiscard]] std::array<T, N> GetComponents() const {
        return m_components;
    }

    void SetComponents(const std::array<T, N>& values) {
        m_components = values;
    }

    T& operator[](std::size_t index) {
        assert(index < N);
        return m_components[index];
    }

    const T& operator[](std::size_t index) const {
        assert(index < N);
        return m_components[index];
    }

    // ---- Arithmetic operators (vector op scalar) ----------------------------

    [[nodiscard]] Vector operator+(const T t) const {
        Vector result = *this;
        result += t;
        return result;
    }

    [[nodiscard]] Vector operator-(const T t) const {
        Vector result = *this;
        result -= t;
        return result;
    }

    [[nodiscard]] Vector operator*(const T t) const {
        Vector result = *this;
        result *= t;
        return result;
    }

    [[nodiscard]] Vector operator/(const T t) const {
        Vector result = *this;
        result /= t;
        return result;
    }

    // ---- Arithmetic operators (vector op vector) ----------------------------
    //
    // Note: operator*(Vector) and operator/(Vector) perform element-wise
    // (Hadamard) multiplication/division. For the geometric dot product, use
    // the free function DotProduct(); for the cross product, CrossProduct().

    [[nodiscard]] Vector operator+(const Vector& v) const {
        Vector result = *this;
        result += v;
        return result;
    }

    [[nodiscard]] Vector operator-(const Vector& v) const {
        Vector result = *this;
        result -= v;
        return result;
    }

    /** @brief Element-wise (Hadamard) product. */
    [[nodiscard]] Vector operator*(const Vector& v) const {
        Vector result = *this;
        result *= v;
        return result;
    }

    /** @brief Element-wise (Hadamard) division. */
    [[nodiscard]] Vector operator/(const Vector& v) const {
        Vector result = *this;
        result /= v;
        return result;
    }

    // ---- Compound assignment (scalar) ---------------------------------------

    Vector& operator+=(const T t) {
        for (auto& c : m_components)
            c += t;
        return *this;
    }

    Vector& operator-=(const T t) {
        for (auto& c : m_components)
            c -= t;
        return *this;
    }

    Vector& operator*=(const T t) {
        for (auto& c : m_components)
            c *= t;
        return *this;
    }

    Vector& operator/=(const T t) {
        if constexpr (std::is_floating_point<T>::value) {
            assert(std::abs(t) >
                   std::numeric_limits<T>::epsilon() * static_cast<T>(10));
        } else {
            assert(t != static_cast<T>(0));
        }
        for (auto& c : m_components)
            c /= t;
        return *this;
    }

    // ---- Compound assignment (vector) ---------------------------------------

    Vector& operator+=(const Vector& v) {
        for (std::size_t i = 0; i < N; ++i)
            m_components[i] += v[i];
        return *this;
    }

    Vector& operator-=(const Vector& v) {
        for (std::size_t i = 0; i < N; ++i)
            m_components[i] -= v[i];
        return *this;
    }

    /** @brief Element-wise (Hadamard) multiply-assign. */
    Vector& operator*=(const Vector& v) {
        for (std::size_t i = 0; i < N; ++i)
            m_components[i] *= v[i];
        return *this;
    }

    /** @brief Element-wise (Hadamard) divide-assign. */
    Vector& operator/=(const Vector& v) {
        for (std::size_t i = 0; i < N; ++i) {
            if constexpr (std::is_floating_point<T>::value) {
                assert(std::abs(v[i]) >
                       std::numeric_limits<T>::epsilon() * static_cast<T>(10));
            } else {
                assert(v[i] != static_cast<T>(0));
            }
            m_components[i] /= v[i];
        }
        return *this;
    }

    // ---- Geometric operations -----------------------------------------------

    /** @brief Returns the squared Euclidean length of the vector. */
    [[nodiscard]] T MagnitudeSquared() const {
        return std::inner_product(m_components.begin(),
                                  m_components.end(),
                                  m_components.begin(),
                                  static_cast<T>(0));
    }

    /** @brief Returns the Euclidean length (magnitude) of the vector. */
    [[nodiscard]] T Magnitude() const {
        return std::sqrt(MagnitudeSquared());
    }

    /**
     * @brief Returns a normalized (unit-length) copy of this vector.
     *
     * If the vector is zero-length, returns it unchanged.
     */
    [[nodiscard]] Vector Normalize() const {
        const T len = Magnitude();
        if constexpr (std::is_floating_point<T>::value) {
            if (len > std::numeric_limits<T>::epsilon())
                return *this / len;
        } else {
            if (len != static_cast<T>(0))
                return *this / len;
        }
        return *this;
    }

    /**
     * @brief Normalizes the vector in-place.
     *
     * If the vector is zero-length, leaves it unchanged.
     * @return Reference to *this.
     */
    Vector& NormalizeInPlace() {
        const T len = Magnitude();
        if constexpr (std::is_floating_point<T>::value) {
            if (len > std::numeric_limits<T>::epsilon())
                *this /= len;
        } else {
            if (len != static_cast<T>(0))
                *this /= len;
        }
        return *this;
    }

    /**
     * @brief Inverts (negates) every component in-place.
     * @return Reference to *this.
     */
    Vector& Invert() {
        for (auto& c : m_components)
            c = -c;
        return *this;
    }

    // ---- Utilities ----------------------------------------------------------

    /** @brief Prints the vector to std::cout. */
    void Print() const {
        std::cout << "[Vector] - " << *this << "\n";
    }

   private:
    std::array<T, N> m_components;
};

// ==============================
// Non-member functions
// ==============================

/**
 * @brief 3D cross product.
 *
 * Operands are cast to the common type *before* multiplication to avoid
 * precision loss when T and T2 differ.
 */
template <typename T, typename T2, std::size_t N>
auto CrossProduct(const Vector<T, N>& v1, const Vector<T2, N>& v2)
    -> Vector<typename std::common_type<T, T2>::type, N> {
    static_assert(N == 3,
                  "CrossProduct is only defined for 3-dimensional vectors.");
    using R = typename std::common_type<T, T2>::type;

    // Cast individual components first so arithmetic is done in type R.
    const R a0 = static_cast<R>(v1[0]), a1 = static_cast<R>(v1[1]),
            a2 = static_cast<R>(v1[2]);
    const R b0 = static_cast<R>(v2[0]), b1 = static_cast<R>(v2[1]),
            b2 = static_cast<R>(v2[2]);

    return Vector<R, N>{
        a1 * b2 - a2 * b1, a2 * b0 - a0 * b2, a0 * b1 - a1 * b0};
}

/**
 * @brief Dot (inner) product of two vectors.
 *
 * Operands are cast to the common type before multiplication.
 */
template <typename T, typename T2, std::size_t N>
auto DotProduct(const Vector<T, N>& v1, const Vector<T2, N>& v2) ->
    typename std::common_type<T, T2>::type {
    using R = typename std::common_type<T, T2>::type;
    R result = static_cast<R>(0);
    for (std::size_t i = 0; i < N; ++i)
        result += static_cast<R>(v1[i]) * static_cast<R>(v2[i]);
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
    Vector<T, N> result = v;
    result.Invert();
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
// Convenience type aliases
// ==============================
//
// Explicit instantiations belong in a .cpp file.
// These aliases are the public API for common specializations.

using Vec2f = Vector<float, 2>;
using Vec3f = Vector<float, 3>;
using Vec4f = Vector<float, 4>;
using Vec2d = Vector<double, 2>;
using Vec3d = Vector<double, 3>;
using Vec4d = Vector<double, 4>;

}  // namespace RTB

#endif /* VECTOR_HPP */