#ifndef PATHPLANNING_JMT_H
#define PATHPLANNING_JMT_H

#include <memory>
#include <unordered_map>

#include "math.h"
#include "path.h"
#include "tracker.h"
#include "vehicle.h"

namespace pathplanning {

/* -------------------------------------------------------------------------- */
//
// JMT Solvers
//
/* -------------------------------------------------------------------------- */

/**
 * @brief      This class describes a sd waypoint function.
 */
class SDFunctor {
 public:
  SDFunctor() {}

  SDFunctor(const QuinticFunctor& sFunc, const QuinticFunctor& dFunc)
      : _sPosFunc(sFunc), _dPosFunc(dFunc) {
    _sVelFunc = _sPosFunc.Differentiate();
    _dVelFunc = _dPosFunc.Differentiate();
    _sAccFunc = _sVelFunc.Differentiate();
    _dAccFunc = _dVelFunc.Differentiate();
  }

  std::array<double, 6> Eval(const double x) const {
    return {_sPosFunc(x), _sVelFunc(x), _sAccFunc(x),
            _dPosFunc(x), _dVelFunc(x), _dAccFunc(x)};
  }

  std::array<double, 6> operator()(const double x) const { return Eval(x); }

  const QuinticFunctor& GetSFunc() const { return _sPosFunc; }
  const QuinticFunctor& GetDFunc() const { return _dPosFunc; }

 private:
  QuinticFunctor _sPosFunc;
  QuarticFunctor _sVelFunc;
  CubicFunctor _sAccFunc;
  QuinticFunctor _dPosFunc;
  QuarticFunctor _dVelFunc;
  CubicFunctor _dAccFunc;
};

/**
 * @brief      JMT Trajectory representation
 *
 * JMT trajectory is used for producing a waypoint at a given time.
 *
 * This trajectory is also flexible for evaluation since it preserves a lot of
 * useful information than the final product which is a set of discrete
 * waypoints.
 *
 * Note that not all trajectory generators use the same information to produce
 * the same format of output, so trajecotry is per generator type.
 */
struct JMTTrajectory {
  SDFunctor _sdFunc;
  double elapsedTime = 0.0;

  JMTTrajectory() {}

  JMTTrajectory(const SDFunctor& sdFunc, const double elapsedTime)
      : _sdFunc(sdFunc), elapsedTime(elapsedTime) {}

  inline VehicleConfiguration Eval(const double t) const {
    return VehicleConfiguration(_sdFunc(t));
  }

  VehicleConfiguration operator()(const double t) const { return Eval(t); }

  /**
   * @brief      Get nearest approach from trajectory to predicted vehicle
   * position
   *
   * @param[in]  v                Vehicle
   * @param[in]  maxTimeDuration  The maximum time duration
   * @param[in]  timeStep         The time step for checking
   *
   * @return     distance in meter
   */
  double ComputeNearestApproach(const Vehicle& vehicle, double maxTimeDuration,
                                double timeStep);

  double ComputeNearestApproach(const std::vector<Vehicle>& vehicles,
                                double maxTimeDuration, double timeStep);
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
  static QuinticFunctor Solve1D(const std::array<double, 3>& start,
                                const std::array<double, 3>& end,
                                const double t);

  static QuinticFunctor Solve1D(const std::array<double, 6>& params,
                                const double t);

  /**
   * @brief      Compute a SDFunctor
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
  static SDFunctor Solve2D(const std::array<double, 6>& sParams,
                           const std::array<double, 6>& dParams,
                           const double t);

  static JMTTrajectory ComputeTrajectory(const std::array<double, 6>& sParams,
                                         const std::array<double, 6>& dParams,
                                         const double t);

  static JMTTrajectory ComputeTrajectory(const VehicleConfiguration& start,
                                         const VehicleConfiguration& end,
                                         const double t);
};

}  // namespace pathplanning

inline std::ostream& operator<<(std::ostream& out,
                                const pathplanning::SDFunctor& functor) {
  return out << fmt::format("SDFunctor(QuinticFunctor({}), QuinticFunctor({}))",
                            functor.GetSFunc(), functor.GetDFunc());
}

inline std::ostream& operator<<(std::ostream& out,
                                const pathplanning::JMTTrajectory& traj) {
  return out << fmt::format("JMTTrajectory({}, {})", traj.elapsedTime,
                            traj._sdFunc);
}

#endif
