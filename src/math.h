#ifndef PATHPLANNING_MATH_H
#define PATHPLANNING_MATH_H

#include <cmath>
#include <random>

#include "log.h"

namespace pathplanning {

inline double
mph2ms(const double mph)
{
  return mph / 2.24;
}
inline double
ms2mph(const double ms)
{
  return ms * 2.24;
}

inline double
CalculateVelocity(double vel, double acc, double time)
{
  return vel + acc * time;
}

inline double
CalculatePosition(double pos, double vel, double acc, double time)
{
  return pos + vel * time + acc * time * time / 2.0;
}

inline double
deg2rad(double x)
{
  return x * M_PI / 180.0;
}
inline double
rad2deg(double x)
{
  return x * 180.0 / M_PI;
}

inline double
GetDistance(double x1, double y1, double x2, double y2)
{
  return std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

inline double
GetDistance(std::array<double, 2> p1, std::array<double, 2> p2)
{
  return GetDistance(p1[0], p1[1], p2[0], p2[1]);
}

inline double
Logistic(const double x)
{
  return 2.0 / (1.0 + std::exp(-x)) - 1.0;
}

/**
 * @brief      Polynomial equation definition
 */
template<uint32_t Order>
struct PolynomialFunctor
{
  PolynomialFunctor() {}
  PolynomialFunctor(const std::array<double, Order + 1>& coeffs)
    : coeffs(coeffs)
  {}

  inline double Eval(const double x) const
  {
    double item = 1.0;
    double result = 0.0;
    for (size_t i = 0; i < coeffs.size(); ++i) {
      result += coeffs[i] * item;
      item *= x;
    }
    return result;
  }

  double operator()(const double x) const { return Eval(x); }

  inline PolynomialFunctor<Order - 1> Differentiate() const
  {
    std::array<double, Order> newCoeffs;
    for (size_t i = 0; i < coeffs.size() - 1; ++i) {
      newCoeffs[i] = coeffs[i + 1] * (i + 1);
    }
    return PolynomialFunctor<Order - 1>(newCoeffs);
  }

  std::array<double, Order + 1> coeffs;
};

using LinearFunctor = PolynomialFunctor<1>;
using QuadraticFunctor = PolynomialFunctor<2>;
using CubicFunctor = PolynomialFunctor<3>;
using QuarticFunctor = PolynomialFunctor<4>;
using QuinticFunctor = PolynomialFunctor<5>;

template<typename T = double>
struct GaussianSampler1D
{
  GaussianSampler1D(const T mu, const T sigma)
  {
    gen = std::mt19937(rd());
    dist = std::normal_distribution<T>(mu, sigma);
  }

  double Sample() { return dist(gen); }

  std::random_device rd;
  std::mt19937 gen;
  std::normal_distribution<T> dist;
};

} // namespace pathplanning

template<uint32_t Order>
inline std::ostream&
operator<<(std::ostream& out,
           const pathplanning::PolynomialFunctor<Order>& functor)
{
  return out << functor.coeffs;
}

#endif
