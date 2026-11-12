/*
 ____ _____ ____
|  _ \_   _| __ )
| |_) || | |  _ \
|  _ < | | | |_) |
|_| \_\|_| |____/

 Ellipse.hpp
 2D ellipse: parametrization, arc length, and circumference.

 Copyright (c) 2026 Robin Breugelmans
 Licensed under the MIT License. See LICENSE in the project root.
*/

#ifndef RTB_ELLIPSE_HPP
#define RTB_ELLIPSE_HPP

#include <RTB/Math.hpp>
#include <RTB/Point.hpp>
#include <RTB/Vector.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>

namespace RTB {

template <typename T>
class Ellipse {
   public:
    Ellipse() = default;

    Ellipse(const Point<T, 3>& center,
            const std::array<Vector<T, 3>, 2>& semi_axes,
            const std::array<T, 2>& semi_axis_lengths,
            const Vector<T, 3>& normal)
        : m_center(center),
          m_semi_axes(semi_axes),
          m_semi_axis_lengths(semi_axis_lengths),
          m_normal(normal) {}

    [[nodiscard]] T arcLength(const Point<T, 3>& p1,
                              const Point<T, 3>& p2) const;

    [[nodiscard]] T circumference() const;

    [[nodiscard]] const Point<T, 3>& getCenter() const {
        return m_center;
    }

    [[nodiscard]] const std::array<Vector<T, 3>, 2>& getSemiAxes() const {
        return m_semi_axes;
    }

    [[nodiscard]] const std::array<T, 2>& getSemiAxisLengths() const {
        return m_semi_axis_lengths;
    }

    [[nodiscard]] const Vector<T, 3>& getNormal() const {
        return m_normal;
    }

   private:
    Point<T, 3> m_center{};
    std::array<Vector<T, 3>, 2> m_semi_axes{};  // orthonormal basis
    std::array<T, 2> m_semi_axis_lengths{};     // a (major), b (minor)
    Vector<T, 3> m_normal{};

    [[nodiscard]] T parameterOf(const Point<T, 3>& point) const;

    static T integrand(T angle, T semi_major, T semi_minor);
    static T gaussLegendre16(T angle_0, T angle_1, T semi_major, T semi_minor);
};

// ------------------------------------------------------------
// Robust parameterization in ellipse frame
// ------------------------------------------------------------
template <typename T>
T Ellipse<T>::parameterOf(const Point<T, 3>& point) const {
    const Vector<T, 3> delta(m_center, point);

    const T proj_x = dotProduct(delta, m_semi_axes[0]);
    const T proj_y = dotProduct(delta, m_semi_axes[1]);

    // stable ellipse parameterization
    T angle = std::atan2(proj_y, proj_x);

    if (angle < T(0)) {
        angle += T(2) * pi;
    }

    return angle;
}

// ------------------------------------------------------------
// Arc-length integrand
// ds = sqrt(a^2 sin^2 t + b^2 cos^2 t)
// ------------------------------------------------------------
template <typename T>
T Ellipse<T>::integrand(T angle, T semi_major, T semi_minor) {
    const T sin_t = std::sin(angle);
    const T cos_t = std::cos(angle);

    return std::sqrt(semi_major * semi_major * sin_t * sin_t +
                     semi_minor * semi_minor * cos_t * cos_t);
}

// ------------------------------------------------------------
// Clean Gauss-Legendre integration on ordered interval only
// ------------------------------------------------------------
template <typename T>
T Ellipse<T>::gaussLegendre16(T angle_0,
                              T angle_1,
                              T semi_major,
                              T semi_minor) {
    if (std::abs(angle_1 - angle_0) < std::numeric_limits<T>::epsilon()) {
        return T(0);
    }

    if (angle_1 < angle_0) {
        std::swap(angle_0, angle_1);
    }

    static constexpr std::array<double, 8> nodes = {
        0.09501250983763744,
        0.28160355077925891,
        0.45801677765722739,
        0.61787624440264375,
        0.75540440835500303,
        0.86563120238783174,
        0.94457502307323258,
        0.98940093499164993,
    };

    static constexpr std::array<double, 8> weights = {
        0.18945061045506850,
        0.18260341504492359,
        0.16915651939500254,
        0.14959598881657673,
        0.12462897125553387,
        0.09515851168249278,
        0.06225352393864789,
        0.02715245941175409,
    };

    const T mid = (angle_0 + angle_1) / 2;
    const T half = (angle_1 - angle_0) / 2;

    T sum = 0;

    for (std::size_t i = 0; i < nodes.size(); ++i) {
        const T delta_angle = half * static_cast<T>(nodes[i]);
        const T weight = static_cast<T>(weights[i]);

        sum += weight * (integrand(mid - delta_angle, semi_major, semi_minor) +
                         integrand(mid + delta_angle, semi_major, semi_minor));
    }

    return half * sum;
}

// ------------------------------------------------------------
// Circumference (stable reference length)
// ------------------------------------------------------------
template <typename T>
T Ellipse<T>::circumference() const {
    const T semi_major = m_semi_axis_lengths[0];
    const T semi_minor = m_semi_axis_lengths[1];

    if (std::abs(semi_major - semi_minor) < std::numeric_limits<T>::epsilon()) {
        return T(2) * pi * semi_major;
    }

    return gaussLegendre16(T(0), T(2) * pi, semi_major, semi_minor);
}

// ------------------------------------------------------------
// Correct shortest-arc selection
// ------------------------------------------------------------
template <typename T>
T Ellipse<T>::arcLength(const Point<T, 3>& p1, const Point<T, 3>& p2) const {
    const T semi_major = m_semi_axis_lengths[0];
    const T semi_minor = m_semi_axis_lengths[1];

    const T two_pi = T(2) * pi;

    T angle_1 = parameterOf(p1);
    T angle_2 = parameterOf(p2);

    // forward direction
    T forward_start = angle_1;
    T forward_end = angle_2;
    if (forward_end < forward_start) {
        forward_end += two_pi;
    }

    T forward =
        gaussLegendre16(forward_start, forward_end, semi_major, semi_minor);

    // backward direction (explicit recomputation)
    T backward_start = angle_2;
    T backward_end = angle_1;
    if (backward_end < backward_start) {
        backward_end += two_pi;
    }

    T backward =
        gaussLegendre16(backward_start, backward_end, semi_major, semi_minor);

    // choose shorter
    return std::min(forward, backward);
}

}  // namespace RTB

#endif  // RTB_ELLIPSE_HPP