#ifndef ELLIPSE_HPP
#define ELLIPSE_HPP

#include <RTB/Math.hpp>
#include <RTB/Point.hpp>
#include <RTB/Vector.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>

namespace RTB {

template <typename T>
class Ellipse {
   public:
    [[nodiscard]] T arcLength(const Point<T, 3>& p1,
                              const Point<T, 3>& p2) const;

    [[nodiscard]] T circumference() const;

    Point<T, 3> center{};
    std::array<Vector<T, 3>, 2> semi_axes{};  // orthonormal basis
    std::array<T, 2> semi_axis_lengths{};     // a (major), b (minor)
    Vector<T, 3> normal{};

   private:
    [[nodiscard]] T parameterOf(const Point<T, 3>& p) const;

    static T integrand(T t, T a, T b);
    static T gaussLegendre16(T t0, T t1, T a, T b);
};

// ------------------------------------------------------------
// Robust parameterization in ellipse frame
// ------------------------------------------------------------
template <typename T>
T Ellipse<T>::parameterOf(const Point<T, 3>& p) const {
    const T a = semi_axis_lengths[0];
    const T b = semi_axis_lengths[1];

    const Vector<T, 3> d(center, p);

    const T x = dotProduct(d, semi_axes[0]);
    const T y = dotProduct(d, semi_axes[1]);

    // stable ellipse parameterization
    T t = std::atan2(y, x);

    if (t < T(0)) {
        t += T(2) * pi;
    }

    return t;
}

// ------------------------------------------------------------
// Arc-length integrand
// ds = sqrt(a^2 sin^2 t + b^2 cos^2 t)
// ------------------------------------------------------------
template <typename T>
T Ellipse<T>::integrand(T t, T a, T b) {
    const T s = std::sin(t);
    const T c = std::cos(t);

    return std::sqrt(a * a * s * s + b * b * c * c);
}

// ------------------------------------------------------------
// Clean Gauss-Legendre integration on ordered interval only
// ------------------------------------------------------------
template <typename T>
T Ellipse<T>::gaussLegendre16(T t0, T t1, T a, T b) {
    if (t0 == t1)
        return T(0);

    if (t1 < t0)
        std::swap(t0, t1);

    static constexpr double nodes[8] = {0.09501250983763744,
                                        0.28160355077925891,
                                        0.45801677765722739,
                                        0.61787624440264375,
                                        0.75540440835500303,
                                        0.86563120238783174,
                                        0.94457502307323258,
                                        0.98940093499164993};

    static constexpr double weights[8] = {0.18945061045506850,
                                          0.18260341504492359,
                                          0.16915651939500254,
                                          0.14959598881657673,
                                          0.12462897125553387,
                                          0.09515851168249278,
                                          0.06225352393864789,
                                          0.02715245941175409};

    const T mid = (t0 + t1) / 2;
    const T half = (t1 - t0) / 2;

    T sum = 0;

    for (int i = 0; i < 8; ++i) {
        const T dx = half * static_cast<T>(nodes[i]);
        const T w = static_cast<T>(weights[i]);

        sum += w * (integrand(mid - dx, a, b) + integrand(mid + dx, a, b));
    }

    return half * sum;
}

// ------------------------------------------------------------
// Circumference (stable reference length)
// ------------------------------------------------------------
template <typename T>
T Ellipse<T>::circumference() const {
    const T a = semi_axis_lengths[0];
    const T b = semi_axis_lengths[1];

    if (std::abs(a - b) < std::numeric_limits<T>::epsilon()) {
        return T(2) * pi * a;
    }

    return gaussLegendre16(T(0), T(2) * pi, a, b);
}

// ------------------------------------------------------------
// Correct shortest-arc selection
// ------------------------------------------------------------
template <typename T>
T Ellipse<T>::arcLength(const Point<T, 3>& p1, const Point<T, 3>& p2) const {
    const T a = semi_axis_lengths[0];
    const T b = semi_axis_lengths[1];

    const T two_pi = T(2) * pi;

    T t1 = parameterOf(p1);
    T t2 = parameterOf(p2);

    // forward direction
    T f0 = t1;
    T f1 = t2;
    if (f1 < f0)
        f1 += two_pi;

    T forward = gaussLegendre16(f0, f1, a, b);

    // backward direction (explicit recomputation)
    T b0 = t2;
    T b1 = t1;
    if (b1 < b0)
        b1 += two_pi;

    T backward = gaussLegendre16(b0, b1, a, b);

    // choose shorter
    return std::min(forward, backward);
}

}  // namespace RTB

#endif