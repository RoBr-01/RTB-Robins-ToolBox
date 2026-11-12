#include <RTB/Colour.hpp>
#include <RTB/Ellipse.hpp>
#include <RTB/Ellipsoid.hpp>
#include <RTB/Plane.hpp>
#include <RTB/Point.hpp>
#include <RTB/Ray.hpp>
#include <RTB/Vector.hpp>

template class RTB::Colour<float, 4>;
template class RTB::Colour<double, 4>;
template class RTB::Ellipsoid<float>;
template class RTB::Ellipsoid<double>;
template class RTB::Ellipse<double>;
template class RTB::Plane<float>;
template class RTB::Plane<double>;
template class RTB::Point<double, 3>;
template class RTB::Ray<double, 3>;
template class RTB::Vector<double, 3>;

int main() {}
