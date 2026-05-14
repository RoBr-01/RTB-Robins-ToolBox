#ifndef VECTOR_HPP
#define VECTOR_HPP

// STL
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <initializer_list>
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
auto crossProduct(const Vector<T, N>& vector_1, const Vector<T2, N>& vector_2)
    -> Vector<std::common_type_t<T, T2>, N>;

template <typename T, typename T2, std::size_t N>
auto dotProduct(const Vector<T, N>& vector_1, const Vector<T2, N>& vector_2)
    -> std::common_type_t<T, T2>;

template <typename T, std::size_t N>
std::ostream& operator<<(std::ostream& out, const Vector<T, N>& vector);

template <typename T, std::size_t N>
Vector<T, N> operator+(const T& scalar, const Vector<T, N>& vector);

template <typename T, std::size_t N>
Vector<T, N> operator-(const T& scalar, const Vector<T, N>& vector);

template <typename T, std::size_t N>
Vector<T, N> operator*(const T& scalar, const Vector<T, N>& vector);

template <typename T, std::size_t N>
Vector<T, N> operator-(const Vector<T, N>& vector);

template <typename T, std::size_t N>
bool operator==(const Vector<T, N>& left_vector,
                const Vector<T, N>& right_vector);

template <typename T, std::size_t N>
bool operator!=(const Vector<T, N>& left_vector,
                const Vector<T, N>& right_vector);

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
    static_assert(std::is_arithmetic_v<T>,
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
        for (std::size_t i = 0; i < N; ++i) {
            m_components[i] = endPoint[i] - startPoint[i];
        }
    }

    explicit Vector(const Point<T, N>& endPoint) {
        for (std::size_t i = 0; i < N; ++i) {
            m_components[i] = endPoint[i];
        }
    }

    // ---- Accessors ----------------------------------------------------------

    static constexpr std::size_t size() {
        return N;
    }

    [[nodiscard]] std::array<T, N> getComponents() const {
        return m_components;
    }

    void setComponents(const std::array<T, N>& values) {
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

    [[nodiscard]] Vector operator+(const T scalar) const {
        Vector result = *this;
        result += scalar;
        return result;
    }

    [[nodiscard]] Vector operator-(const T scalar) const {
        Vector result = *this;
        result -= scalar;
        return result;
    }

    [[nodiscard]] Vector operator*(const T scalar) const {
        Vector result = *this;
        result *= scalar;
        return result;
    }

    [[nodiscard]] Vector operator/(const T scalar) const {
        Vector result = *this;
        result /= scalar;
        return result;
    }

    // ---- Arithmetic operators (vector op vector) ----------------------------
    //
    // Note: operator*(Vector) and operator/(Vector) perform element-wise
    // (Hadamard) multiplication/division. For the geometric dot product, use
    // the free function dotProduct(); for the cross product, crossProduct().

    [[nodiscard]] Vector operator+(const Vector& vector) const {
        Vector result = *this;
        result += vector;
        return result;
    }

    [[nodiscard]] Vector operator-(const Vector& vector) const {
        Vector result = *this;
        result -= vector;
        return result;
    }

    /** @brief Element-wise (Hadamard) product. */
    [[nodiscard]] Vector operator*(const Vector& vector) const {
        Vector result = *this;
        result *= vector;
        return result;
    }

    /** @brief Element-wise (Hadamard) division. */
    [[nodiscard]] Vector operator/(const Vector& vector) const {
        Vector result = *this;
        result /= vector;
        return result;
    }

    // ---- Compound assignment (scalar) ---------------------------------------

    Vector& operator+=(const T scalar) {
        for (auto& component : m_components) {
            component += scalar;
        }
        return *this;
    }

    Vector& operator-=(const T scalar) {
        for (auto& component : m_components) {
            component -= scalar;
        }
        return *this;
    }

    Vector& operator*=(const T scalar) {
        for (auto& component : m_components) {
            component *= scalar;
        }
        return *this;
    }

    Vector& operator/=(const T scalar) {
        if constexpr (std::is_floating_point_v<T>) {
            assert(std::abs(scalar) >
                   std::numeric_limits<T>::epsilon() * static_cast<T>(10));
        } else {
            assert(scalar != static_cast<T>(0));
        }
        for (auto& component : m_components) {
            component /= scalar;
        }
        return *this;
    }

    // ---- Compound assignment (vector) ---------------------------------------

    Vector& operator+=(const Vector& vector) {
        for (std::size_t i = 0; i < N; ++i) {
            m_components[i] += vector[i];
        }
        return *this;
    }

    Vector& operator-=(const Vector& vector) {
        for (std::size_t i = 0; i < N; ++i) {
            m_components[i] -= vector[i];
        }
        return *this;
    }

    /** @brief Element-wise (Hadamard) multiply-assign. */
    Vector& operator*=(const Vector& vector) {
        for (std::size_t i = 0; i < N; ++i) {
            m_components[i] *= vector[i];
        }
        return *this;
    }

    /** @brief Element-wise (Hadamard) divide-assign. */
    Vector& operator/=(const Vector& vector) {
        for (std::size_t i = 0; i < N; ++i) {
            if constexpr (std::is_floating_point_v<T>) {
                assert(std::abs(vector[i]) >
                       std::numeric_limits<T>::epsilon() * static_cast<T>(10));
            } else {
                assert(vector[i] != static_cast<T>(0));
            }
            m_components[i] /= vector[i];
        }
        return *this;
    }

    // ---- Geometric operations -----------------------------------------------

    /** @brief Returns the squared Euclidean length of the vector. */
    [[nodiscard]] T magnitudeSquared() const {
        return std::inner_product(m_components.begin(),
                                  m_components.end(),
                                  m_components.begin(),
                                  static_cast<T>(0));
    }

    /** @brief Returns the Euclidean length (magnitude) of the vector. */
    [[nodiscard]] T magnitude() const {
        return static_cast<T>(std::sqrt(magnitudeSquared()));
    }

    /**
     * @brief Returns a normalized (unit-length) copy of this vector.
     *
     * If the vector is zero-length, returns it unchanged.
     */
    [[nodiscard]] Vector normalize() const {
        const T len = magnitude();
        if constexpr (std::is_floating_point_v<T>) {
            if (len > std::numeric_limits<T>::epsilon()) {
                return *this / len;
            }
        } else {
            if (len != static_cast<T>(0)) {
                return *this / len;
            }
        }
        return *this;
    }

    /**
     * @brief Normalizes the vector in-place.
     *
     * If the vector is zero-length, leaves it unchanged.
     * @return Reference to *this.
     */
    Vector& normalizeInPlace() {
        const T len = magnitude();
        if constexpr (std::is_floating_point_v<T>) {
            if (len > std::numeric_limits<T>::epsilon()) {
                *this /= len;
            }
        } else {
            if (len != static_cast<T>(0)) {
                *this /= len;
            }
        }
        return *this;
    }

    /**
     * @brief Inverts (negates) every component in-place.
     * @return Reference to *this.
     */
    Vector& invert() {
        for (auto& component : m_components) {
            component = -component;
        }
        return *this;
    }

    // ---- Utilities ----------------------------------------------------------

    /** @brief Prints the vector to std::cout. */
    void print() const {
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
auto crossProduct(const Vector<T, N>& vector_1, const Vector<T2, N>& vector_2)
    -> Vector<std::common_type_t<T, T2>, N> {
    static_assert(N == 3,
                  "crossProduct is only defined for 3-dimensional vectors.");
    using R = typename std::common_type<T, T2>::type;

    // Cast individual components first so arithmetic is done in type R.
    const R vector_1_1 = static_cast<R>(vector_1[0]);
    const R vector_1_2 = static_cast<R>(vector_1[1]);
    const R vector_1_3 = static_cast<R>(vector_1[2]);
    const R vector_2_1 = static_cast<R>(vector_2[0]);
    const R vector_2_2 = static_cast<R>(vector_2[1]);
    const R vector_2_3 = static_cast<R>(vector_2[2]);

    return Vector<R, N>{vector_1_2 * vector_2_3 - vector_1_3 * vector_2_2,
                        vector_1_3 * vector_2_1 - vector_1_1 * vector_2_3,
                        vector_1_1 * vector_2_2 - vector_1_2 * vector_2_1};
}

/**
 * @brief Dot (inner) product of two vectors.
 *
 * Operands are cast to the common type before multiplication.
 */
template <typename T, typename T2, std::size_t N>
auto dotProduct(const Vector<T, N>& vector_1, const Vector<T2, N>& vector_2)
    -> std::common_type_t<T, T2> {
    using R = std::common_type_t<T, T2>;
    R result = static_cast<R>(0);
    for (std::size_t i = 0; i < N; ++i) {
        result += static_cast<R>(vector_1[i]) * static_cast<R>(vector_2[i]);
    }
    return result;
}

template <typename T, std::size_t N>
std::ostream& operator<<(std::ostream& out, const Vector<T, N>& vector) {
    out << "(";
    for (std::size_t i = 0; i < N; ++i) {
        out << vector[i];
        if (i < N - 1) {
            out << ", ";
        }
    }
    out << ")";
    return out;
}

template <typename T, std::size_t N>
Vector<T, N> operator+(const T& scalar, const Vector<T, N>& vector) {
    return vector + scalar;
}

template <typename T, std::size_t N>
Vector<T, N> operator-(const T& scalar, const Vector<T, N>& vector) {
    Vector<T, N> result;
    for (std::size_t i = 0; i < N; ++i) {
        result[i] = scalar - vector[i];
    }
    return result;
}

template <typename T, std::size_t N>
Vector<T, N> operator*(const T& scalar, const Vector<T, N>& vector) {
    return vector * scalar;
}

template <typename T, std::size_t N>
Vector<T, N> operator-(const Vector<T, N>& vector) {
    Vector<T, N> result = vector;
    result.invert();
    return result;
}

template <typename T, std::size_t N>
bool operator==(const Vector<T, N>& left_vector,
                const Vector<T, N>& right_vector) {
    return left_vector.getComponents() == right_vector.getComponents();
}

template <typename T, std::size_t N>
bool operator!=(const Vector<T, N>& left_vector,
                const Vector<T, N>& right_vector) {
    return !(left_vector == right_vector);
}

}  // namespace RTB

#endif /* VECTOR_HPP */