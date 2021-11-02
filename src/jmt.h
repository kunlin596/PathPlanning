#ifndef PATHPLANNING_JMT_H
#define PATHPLANNING_JMT_H

#include <memory>
#include <unordered_map>

#include "json.hpp"
#include "math.h"
#include "path.h"
#include "tracker.h"
#include "vehicle.h"

namespace pathplanning {

using Eigen::Matrix62d;
using Eigen::Vector3d;
using Eigen::Vector6d;

/* -------------------------------------------------------------------------- */
//
// JMT Solvers
//
/* -------------------------------------------------------------------------- */

struct JMTTrajectory1d
{
  JMTTrajectory1d() {}

  JMTTrajectory1d(const QuinticFunctor& func, const Vector3d& startCond, const Vector3d& endCond, const double time)
    : _func5(func)
    , _func4(_func5.Differentiate())
    , _func3(_func4.Differentiate())
    , _func2(_func3.Differentiate())
    , _func1(_func2.Differentiate())
    , _func0(_func1.Differentiate())
    , _startCond(startCond)
    , _endCond(endCond)
    , _time(time)
  {}

  inline Vector6d Eval(const double t) const
  {
    Vector6d ret;
    ret << _func5(t), _func4(t), _func3(t), _func2(t), _func1(t), _func0(t);
    return ret;
  }

  inline Vector6d operator()(const double t) const { return Eval(t); }

  const Vector3d& GetStartCond() const { return _startCond; }
  const Vector3d& GetEndCond() const { return _endCond; }

  nlohmann::json Dump() const;
  void Write(const std::string& filename) const;

  const QuinticFunctor& GetFunc5() const { return _func5; }
  const QuarticFunctor& GetFunc4() const { return _func4; }
  const CubicFunctor& GetFunc3() const { return _func3; }
  const QuadraticFunctor& GetFunc2() const { return _func2; }
  const LinearFunctor& GetFunc1() const { return _func1; }
  const ConstantFunctor& GetFunc0() const { return _func0; }
  double GetTime() const { return _time; }

  bool IsValid(const Configuration& conf) const;

private:
  QuinticFunctor _func5;   ///< position
  QuarticFunctor _func4;   ///< velocity
  CubicFunctor _func3;     ///< accelaration
  QuadraticFunctor _func2; ///< jerk
  LinearFunctor _func1;    ///< snap
  ConstantFunctor _func0;  ///< crackle
  Vector3d _startCond;     ///< start condition
  Vector3d _endCond;       ///< end condition
  double _time = 0.0;      ///< trajectory execution time
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
struct JMTTrajectory2d
{

  JMTTrajectory2d(){};

  JMTTrajectory2d(const JMTTrajectory1d& traj1, const JMTTrajectory1d& traj2);

  bool IsValid(const Map& map, const Configuration& conf) const;

  Eigen::Matrix62d Eval(const double t) const;
  Eigen::Matrix62d operator()(const double t) const { return Eval(t); }

  /**
   * @brief      Calculates the nearest approach to given vehicles.
   *
   * @param[in]  vehicle          The target vehicle
   * @param[in]  maxTimeDuration  The maximum time duration for trajectory
   *                              evaluation
   * @param[in]  timeStep         The time step
   *
   * @return     The nearest approach distance.
   */
  double GetNearestApproachTo(const Vehicle& vehicle, double maxTimeDuration, double timeStep) const;

  double GetNearestApproachTo(const std::vector<Vehicle>& vehicles, double maxTimeDuration, double timeStep) const;

  double GetNearestApproachTo(const std::unordered_map<int, Vehicle>& vehicles,
                              double maxTimeDuration,
                              double timeStep) const;

  inline Matrix32d GetStartCond() const
  {
    Matrix32d cond;
    cond.col(0) = _traj1.GetStartCond();
    cond.col(1) = _traj2.GetStartCond();
    return cond;
  }

  inline Matrix32d GetEndCond() const
  {
    Matrix32d cond;
    cond.col(0) = _traj1.GetEndCond();
    cond.col(1) = _traj2.GetEndCond();
    return cond;
  }

  nlohmann::json Dump() const;

  void Write(const std::string& filename) const;

  //
  // Getters
  //

  const JMTTrajectory1d& GetTraj1() const { return _traj1; }
  const JMTTrajectory1d& GetTraj2() const { return _traj2; }
  double GetTime() const { return _traj1.GetTime(); }

  const QuinticFunctor& GetSFunc() const { return _traj1.GetFunc5(); }
  const QuinticFunctor& GetDFunc() const { return _traj2.GetFunc5(); }

  const QuarticFunctor& GetSVelFunc() const { return _traj1.GetFunc4(); }
  const QuarticFunctor& GetDVelFunc() const { return _traj2.GetFunc4(); }

  const CubicFunctor& GetSAccFunc() const { return _traj1.GetFunc3(); }
  const CubicFunctor& GetDAccFunc() const { return _traj2.GetFunc3(); }

  friend std::ostream& operator<<(std::ostream& out, const JMTTrajectory2d& traj);

private:
  JMTTrajectory1d _traj1;
  JMTTrajectory1d _traj2;
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
 *  [    t_f^3 ,      t_f^4,       t_f^5],
 *  [3 * t_f^2 ,  4 * t_f^3,   5 * t_f^4],
 *  [6 * t_f   , 12 * t_f^2,  20 * t_f^3],
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
  static JMTTrajectory1d Solve1D(const Vector6d& conditions, const double t);

  /**
   * @brief      Compute a JMTTrajectory2d
   *
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
  static JMTTrajectory2d Solve2D(const Matrix62d& conditions, const double t);

  /**
   * @brief      Compute set of feasible trajectory
   *
   * @param[in]  sParams  The s parameters
   * @param[in]  dParams  The d parameters
   * @param[in]  conf     The configuration
   *
   * @return     set of feasible trajectories
   */
  static std::vector<JMTTrajectory2d> SolveMultipleFeasible2D(const Matrix62d& conditions,
                                                              const Map& map,
                                                              const Configuration& conf);
};

std::ostream&
operator<<(std::ostream& out, const pathplanning::JMTTrajectory2d& traj);

} // namespace pathplanning

#endif
