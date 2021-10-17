#ifndef PATHPLANNING_JMT_H
#define PATHPLANNING_JMT_H

#include <array>

namespace pathplanning {

/**
 * @brief      5th order polynomial
 */
struct QuinticFunction {
  explicit QuinticFunction(const std::array<double, 6>& coeffs)
      : coeffs(coeffs) {}

  inline double Eval(const double x) {
    double item = 1.0;
    double result = 0.0;
    for (size_t i = 0; i < 6; ++i) {
      result += coeffs[i] * item;
      item *= x;
    }
    return result;
  }

  std::array<double, 6> coeffs;
};

/**
 * @brief      This class describes a sd waypoint function.
 */
class SDWaypointFunction {
 public:
  explicit SDWaypointFunction(const QuinticFunction& sFunc,
                              const QuinticFunction& dFunc)
      : _sFunc(sFunc), _dFunc(dFunc) {}

  std::array<double, 2> Eval(const double x) {
    return { _sFunc.Eval(x), _dFunc.Eval(x) };
  }

 private:
  QuinticFunction _sFunc;
  QuinticFunction _dFunc;
};

/**
 * @brief      JMT generator
 *
 * This class minimizes a 5th order polynomial representing the coordinate
 * w.r.t. time.
 *
 * The 6 coefficients below are the ones we are solving. Giving that we are
 * minimizing the jerk.
 *
 * s(t) =
 *    a0 +
 *    a1 * t +
 *    a2 * t^2 +
 *    a3 * t^3 +
 *    a4 * t^4 +
 *    a5 * t^5
 *
 * By choosing t_i = 0, 6 equations becomes 3 equations.
 *
 * s (t_i) = s (0) = a0
 * s'(t_i) = s'(0) = a1
 * s"(t_i) = s"(0) = 2 * a2
 *
 * s (t) = a3 * t^3 + a4 * t^4 + a5 * t^5                 + C1
 * s'(t) = 3 * a3 * t^2 + 4 * a4 * t^3 + 5 * a5 * t^4     + C2
 * s"(t) = 6 * a3 * t^2 + 12 * a4 * t^2 + 20 * a5 * t^3   + C3
 *
 * Set t = t_f, t_f is the end time stamp.
 *
 * s (t_f) = a3 * t_f^3 + a4 * t_f^4 + a5 * t_f^5                 + C1
 * s'(t_f) = 3 * a3 * t_f^2 + 4 * a4 * t_f^3 + 5 * a5 * t_f^4     + C2
 * s"(t_f) = 6 * a3 * t_f^2 + 12 * a4 * t_f^2 + 20 * a5 * t_f^3   + C3
 *
 * s(t_f), s'(t_f), s"(t_f) and t_f are the boundary conditions set by the user,
 * so they are known quantities.
 *
 * So it becomes a linear system with 3 equations and 3 unknowns.
 *
 * Ax = b
 *
 * where,
 * A = [
 *  [t_f^3,     t_f^4,      t_f^5     ],
 *  [3 * t_f^2, 4 * t_f^3,  5 * t_f^4 ],
 *  [6 * t_f,   12 * t_f^2, 20 * t_f^3],
 * ]
 *
 * x = [a3, a4, a5]
 *
 * b = [s(t_f), s'(t_f), s"(t_f)]
 *
 * The solution above is for 1D JMT for s, the same logic applies to d.
 *
 * Combine these 2 resulrs, we get a real 2D drivable trajectory.
 *
 */
struct JMT {
  /**
   * @brief      Compute a SDWaypointFunction
   *
   * S paramerers, [s(i), s'(i), s"(i), s(f), s'(f), * s"(f)]
   * D parameters, [d(i), d'(i), d"(i), d(f), d'(f), * d"(f)]
   *
   * @param[in]  sParams     The s parameters,
   * @param[in]  dParams     The d parameters
   * @param[in]  t           The target time to reach the goal
   *
   * @return     The sd waypoint function.
   */
  static SDWaypointFunction Solve2D(const std::array<double, 6>& sParams,
                                    const std::array<double, 6>& dParams,
                                    const double t);

  static QuinticFunction Solve1D(const std::array<double, 6>& params,
                                 const double t);
};

}  // namespace pathplanning

#endif
