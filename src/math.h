#ifndef PATHPLANNING_MATH_H
#define PATHPLANNING_MATH_H

#include <cmath>
#include <random>

namespace pathplanning {

inline double CalculateVelocity(double vel, double acc, double time) {
  return vel + acc * time;
}

inline double CalculatePosition(double pos, double vel, double acc, double time) {
  return pos + vel * time + acc * time * time / 2.0;
}

inline double deg2rad(double x) { return x * M_PI / 180.0; }
inline double rad2deg(double x) { return x * 180.0 / M_PI; }

inline double GetDistance(double x1, double y1, double x2, double y2) {
  return std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

inline double Logistic(const double x) {
  return 2.0 / (1.0 + std::exp(-x)) - 1.0;
}

/**
 * @brief      Polynomial equation definition
 */
template <uint32_t Order>
struct PolynomialFunctor {
  PolynomialFunctor() {}
  PolynomialFunctor(const std::array<double, Order + 1>& coeffs)
      : coeffs(coeffs) {}

  inline double Eval(const double x) const {
    double item = 1.0;
    double result = 0.0;
    for (size_t i = 0; i < Order + 1; ++i) {
      result += coeffs[i] * item;
      item *= x;
    }
    return result;
  }

  inline PolynomialFunctor<Order - 1> Differentiate() {
    std::array<double, Order - 1> newCoeffs;
    for (size_t i = 0; i < coeffs.size() - 1; ++i) {
      newCoeffs[i] = coeffs[i + 1] * (i + 1);
    }
    return PolynomialFunctor<Order - 1>(newCoeffs);
  }

  double operator()(const double x) const { return Eval(x); }

  std::array<double, Order + 1> coeffs;
};

using QuinticFunctor = PolynomialFunctor<5>;

template <typename T = double>
struct GaussianSampler1D {
  GaussianSampler1D(const T mu, const T sigma) {
    gen = std::mt19937(rd());
    dist = std::normal_distribution<T>(mu, sigma);
  }

  double Sample() { return dist(gen); }

  std::random_device rd;
  std::mt19937 gen;
  std::normal_distribution<T> dist;
};

}  // namespace pathplanning

#endif
