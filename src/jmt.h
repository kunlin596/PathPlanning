#ifndef PATHPLANNING_JMT_H
#define PATHPLANNING_JMT_H

#include <memory>
#include <unordered_map>

#include "json.hpp"
#include "math.h"
#include "path.h"
#include "tracker.h"
#include "vehicle.h"

namespace Eigen {
using Vector5d = Matrix<double, 5, 1>;
}

namespace pathplanning {

using Eigen::Matrix62d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector5d;
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
    : _positionFn(func)
    , _velocityFn(_positionFn.Differentiate())
    , _accelerationFn(_velocityFn.Differentiate())
    , _jerkFn(_accelerationFn.Differentiate())
    , _snapFn(_jerkFn.Differentiate())
    , _crackleFn(_snapFn.Differentiate())
    , _startCond(startCond)
    , _endCond(endCond)
    , _time(time)
  {}

  JMTTrajectory1d(const QuinticFunctor& func, const Vector3d& startCond, const Vector2d& endCond, const double time)
    : _positionFn(func)
    , _velocityFn(_positionFn.Differentiate())
    , _accelerationFn(_velocityFn.Differentiate())
    , _jerkFn(_accelerationFn.Differentiate())
    , _snapFn(_jerkFn.Differentiate())
    , _crackleFn(_snapFn.Differentiate())
    , _startCond(startCond)
    , _endCond2d(endCond)
    , _time(time)
  {}

  inline Vector6d Eval(const double t) const
  {
    Vector6d ret;
    ret << GetPosition(t), GetVelocity(t), GetAcceleration(t), GetJerk(t), GetSnap(t), GetCrackle(t);
    return ret;
  }

  inline Vector6d operator()(const double t) const { return Eval(t); }

  const Vector3d& GetStartCond() const { return _startCond; }
  const Vector3d& GetEndCond() const { return _endCond; }

  nlohmann::json Dump() const;
  void Write(const std::string& filename) const;

  const QuinticFunctor& GetPositionFn() const { return _positionFn; }
  const QuarticFunctor& GetVelocityFn() const { return _velocityFn; }
  const CubicFunctor& GetAcclerationFn() const { return _accelerationFn; }
  const QuadraticFunctor& GetJerkFn() const { return _jerkFn; }
  const LinearFunctor& GetSnapFn() const { return _snapFn; }
  const ConstantFunctor& GetCrackleFn() const { return _crackleFn; }
  double GetTime() const { return _time; }

  bool IsValid(const Configuration& conf);
  // TODO: Add total acc, jerk
  bool IsValid(double maxVel = Mph2Mps(49.0),
               double maxAcc = 8.0,
               double maxJerk = 8.0,
               double totalAccel = 2.0,
               double totalJerk = 1.0,
               double timeResolution = 0.02);

  double ComputeCost(double kTime, double kPos, double kJerk, double kEfficiency);
  double GetCost() const { return _cost; }

  double GetPosition(double t) const { return _positionFn(t); }
  double GetVelocity(double t) const { return _velocityFn(t); }
  double GetAcceleration(double t) const { return _accelerationFn(t); }
  double GetJerk(double t) const { return _jerkFn(t); }
  double GetSnap(double t) const { return _snapFn(t); }
  double GetCrackle(double t) const { return _snapFn(t); }

  bool GetIsValid() const { return _isvalid; }

private:
  QuinticFunctor _positionFn;   ///< position
  QuarticFunctor _velocityFn;   ///< velocity
  CubicFunctor _accelerationFn; ///< acceleration
  QuadraticFunctor _jerkFn;     ///< jerk
  LinearFunctor _snapFn;        ///< snap
  ConstantFunctor _crackleFn;   ///< crackle
  Vector3d _startCond;          ///< start condition
  Vector3d _endCond;            ///< end condition
  Vector2d _endCond2d;          ///< end condition
  double _time = 0.0;           ///< trajectory execution time
  double _cost = 0.0;           ///< kinematic cost for this trajectory
  bool _isvalid = false;
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
 * the same format of output, so trajectory is per generator type.
 */
struct JMTTrajectory2d
{

  JMTTrajectory2d(){};

  JMTTrajectory2d(const JMTTrajectory1d& traj1, const JMTTrajectory1d& traj2);

  bool IsValid(const Map& map, const Configuration& conf);
  bool GetIsValid() const { return _isvalid; }

  Eigen::Matrix62d Eval(const double t) const;
  Eigen::Matrix62d operator()(const double t) const { return Eval(t); }

  inline Matrix32d GetStartCond() const
  {
    Matrix32d cond;
    cond.col(0) = _lonTraj.GetStartCond();
    cond.col(1) = _latTraj.GetStartCond();
    return cond;
  }

  inline Matrix32d GetEndCond() const
  {
    Matrix32d cond;
    cond.col(0) = _lonTraj.GetEndCond();
    cond.col(1) = _latTraj.GetEndCond();
    return cond;
  }

  nlohmann::json Dump() const;

  void Write(const std::string& filename) const;

  //
  // Getters
  //

  const JMTTrajectory1d& GetLonTraj() const { return _lonTraj; }
  const JMTTrajectory1d& GetLatTraj() const { return _latTraj; }
  double GetTime() const { return std::min(_lonTraj.GetTime(), _latTraj.GetTime()); }

  const QuinticFunctor& GetSFunc() const { return _lonTraj.GetPositionFn(); }
  const QuinticFunctor& GetDFunc() const { return _latTraj.GetPositionFn(); }

  const QuarticFunctor& GetSVelFunc() const { return _lonTraj.GetVelocityFn(); }
  const QuarticFunctor& GetDVelFunc() const { return _latTraj.GetVelocityFn(); }

  const CubicFunctor& GetSAccFunc() const { return _lonTraj.GetAcclerationFn(); }
  const CubicFunctor& GetDAccFunc() const { return _latTraj.GetAcclerationFn(); }

  friend std::ostream& operator<<(std::ostream& out, const JMTTrajectory2d& traj);

private:
  JMTTrajectory1d _lonTraj;
  JMTTrajectory1d _latTraj;
  bool _isvalid = false;
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
  static JMTTrajectory1d Solve1d_6DoF(const Vector6d& conditions, const double t);

  static JMTTrajectory1d Solve1d_5DoF(const Vector5d& conditions, const double t);

  /**
   * @brief      Compute a JMTTrajectory2d
   *
   *
   * S parameters, [s(i), s'(i), s"(i), s(f), s'(f), * s"(f)]
   * D parameters, [d(i), d'(i), d"(i), d(f), d'(f), * d"(f)]
   *
   * @param[in]  sParams     The s parameters,
   * @param[in]  dParams     The d parameters
   * @param[in]  t           The target time to reach the goal
   *
   * @return     The sd waypoint function.
   */
  static JMTTrajectory2d Solve2d(const Matrix62d& conditions, const double t);

  /**
   * @brief      Compute set of feasible trajectory
   *
   * @param[in]  sParams  The s parameters
   * @param[in]  dParams  The d parameters
   * @param[in]  conf     The configuration
   *
   * @return     set of feasible trajectories
   */
  static std::vector<JMTTrajectory2d> SolveMultipleFeasible2d(const Matrix62d& conditions,
                                                              const Map& map,
                                                              const Configuration& conf);
};

std::ostream&
operator<<(std::ostream& out, const pathplanning::JMTTrajectory2d& traj);

} // namespace pathplanning

#endif
