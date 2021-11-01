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

struct JMTTrajectory1D
{

  JMTTrajectory1D() {}

  JMTTrajectory1D(const QuinticFunctor& func, const double time)
    : _func5(func)
    , _func4(_func5.Differentiate())
    , _func3(_func4.Differentiate())
    , _func2(_func3.Differentiate())
    , _func1(_func2.Differentiate())
    , _func0(_func1.Differentiate())
    , _time(time)
  {}

  inline std::array<double, 6> Eval(const double t) const
  {
    return { _func5(t), _func4(t), _func3(t), _func2(t), _func1(t), _func0(t) };
  }

  inline std::array<double, 6> operator()(const double t) const
  {
    return Eval(t);
  }

  const QuinticFunctor& GetFunc5() const { return _func5; }
  const QuarticFunctor& GetFunc4() const { return _func4; }
  const CubicFunctor& GetFunc3() const { return _func3; }
  const QuadraticFunctor& GetFunc2() const { return _func2; }
  const LinearFunctor& GetFunc1() const { return _func1; }
  const ConstantFunctor& GetFunc0() const { return _func0; }
  double GetTime() const { return _time; }

private:
  QuinticFunctor _func5;
  QuarticFunctor _func4;
  CubicFunctor _func3;
  QuadraticFunctor _func2;
  LinearFunctor _func1;
  ConstantFunctor _func0;
  double _time = 0.0;
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
struct JMTTrajectory2D
{
  double elapsedTime = 0.0;

  JMTTrajectory2D() {}

  JMTTrajectory2D(const JMTTrajectory1D& traj1, const JMTTrajectory1D& traj2)
    : _traj1(traj1)
    , _traj2(traj2)
  {
    assert(traj1.GetTime() == traj2.GetTime());
  }

  inline VehicleConfiguration Eval(const double t) const
  {
    return VehicleConfiguration(_traj1.GetFunc5()(t),
                                _traj1.GetFunc4()(t),
                                _traj1.GetFunc3()(t),
                                _traj2.GetFunc5()(t),
                                _traj2.GetFunc4()(t),
                                _traj2.GetFunc3()(t));
  }

  VehicleConfiguration operator()(const double t) const { return Eval(t); }

  const JMTTrajectory1D& GetTraj1() const { return _traj1; }
  const JMTTrajectory1D& GetTraj2() const { return _traj2; }
  double GetTime() const { return _traj1.GetTime(); }

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
  double ComputeNearestApproach(const Vehicle& vehicle,
                                double maxTimeDuration,
                                double timeStep) const;

  double ComputeNearestApproach(const std::vector<Vehicle>& vehicles,
                                double maxTimeDuration,
                                double timeStep) const;

  double ComputeNearestApproach(
    const std::unordered_map<int, Vehicle>& vehicles,
    double maxTimeDuration,
    double timeStep) const;

  const QuinticFunctor& GetSFunc() const { return _traj1.GetFunc5(); }
  const QuinticFunctor& GetDFunc() const { return _traj2.GetFunc5(); }

  const QuarticFunctor& GetSVelFunc() const { return _traj1.GetFunc4(); }
  const QuarticFunctor& GetDVelFunc() const { return _traj2.GetFunc4(); }

  const CubicFunctor& GetSAccFunc() const { return _traj1.GetFunc3(); }
  const CubicFunctor& GetDAccFunc() const { return _traj2.GetFunc3(); }

  friend std::ostream& operator<<(std::ostream& out,
                                  const JMTTrajectory2D& traj);

private:
  JMTTrajectory1D _traj1;
  JMTTrajectory1D _traj2;
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
 * s(t_f), s'(t_f), s"(t_f) and t_f are the boundary conditions set by the
 * user, so they are known quantities.
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
struct JMT
{
  static JMTTrajectory1D Solve1D(const std::array<double, 3>& start,
                                 const std::array<double, 3>& end,
                                 const double t);

  static JMTTrajectory1D Solve1D(const std::array<double, 6>& params,
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
  static JMTTrajectory2D Solve2D(const std::array<double, 6>& sParams,
                                 const std::array<double, 6>& dParams,
                                 const double t);

  static JMTTrajectory2D ComputeTrajectory(const std::array<double, 6>& sParams,
                                           const std::array<double, 6>& dParams,
                                           const double t);

  static JMTTrajectory2D ComputeTrajectory(const VehicleConfiguration& start,
                                           const VehicleConfiguration& end,
                                           const double t);
};

std::ostream&
operator<<(std::ostream& out, const pathplanning::JMTTrajectory2D& traj);

} // namespace pathplanning

#endif
