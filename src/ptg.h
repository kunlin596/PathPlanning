#ifndef PATHPLANNING_PTG_H
#define PATHPLANNING_PTG_H

#include <boost/optional.hpp>

#include "jmt.h"
#include "json.hpp"
#include "tracker.h"
#include "traj_evaluator.h"

namespace pathplanning {

using namespace nlohmann;

enum class LongitudinalManeuverType
{
  kFollowing,
  kVelocityKeeping,
  kStopping,
};

enum class LateralManeuverType
{
  kLaneKeeping,
  kLeftLaneChanging,
  kRightLaneChanging
};

/**
 * @brief      This class describes a polynomial trajectory generator.
 */
class PolynomialTrajectoryGenerator
{
public:
  explicit PolynomialTrajectoryGenerator(const Map& map, const Configuration& conf);
  virtual ~PolynomialTrajectoryGenerator() {}

  JMTTrajectory2d GenerataTrajectory(const Ego& ego,
                                     const TrackedVehicleMap& trackedVehicleMap,
                                     bool usePython = false);

  Matrix32d ComputeStartState(const Vehicle& ego,
                              const JMTTrajectory2d& prevTraj,
                              const Waypoints& prevPath,
                              bool usePython = false);

private:
  /**
   * @brief      Solve for multiple feasible 1D JMT trajectories
   *
   * @param[in]  s0            The s0, part of constraints
   * @param[in]  s0dot         The s0 dot, part of constraints
   * @param[in]  s0ddot        The s0 dot dot, part of constraints
   * @param[in]  s1            The s1, part of constraints
   * @param[in]  s1dot         The s1 dot, part of constraints
   * @param[in]  s1ddot        The s1 dot dot, part of constraints
   * @param[in]  TjList        The tj list, target time list
   * @param[in]  dsList        The ds list, deltas for s1
   * @param[in]  kTime         The weight for time cost
   * @param[in]  kPos          The weight for position cost
   * @param[in]  kVel          The weight for velocity cost
   * @param[in]  kAcc          The weight for acceleration cost
   * @param[in]  kJerk         The weight for jerk cost
   * @param      trajectories  The trajectories
   */
  void _SolveFullConstraints1d(double s0,
                               double s0dot,
                               double s0ddot,
                               double s1,
                               double s1dot,
                               double s1ddot,
                               const std::vector<double>& TjList,
                               const std::vector<double>& dsList,
                               double kTime,
                               double kPos,
                               double kVel,
                               double kAcc,
                               double kJerk,
                               std::vector<JMTTrajectory1d>& trajectories);

  /**
   * @brief      Generate 1d lateral trajectory for 1 lane
   *
   * @param[in]  latBehavior   The lat behavior
   * @param[in]  ego           The ego
   * @param      trajectories  The trajectories
   */
  void _GenerateLatTrajectory(const LateralManeuverType& latBehavior,
                              const Ego& ego,
                              std::vector<JMTTrajectory1d>& trajectories);

  /**
   * @brief      Generate 1d longitudinal trajectory for 1 lane
   *
   * @param[in]  lonBehavior   The lon behavior
   * @param[in]  ego           The ego
   * @param[in]  vehicle       The vehicle
   * @param      trajectories  The trajectories
   */
  void _GenerateLonTrajectory(const LongitudinalManeuverType& lonBehavior,
                              const Ego& ego,
                              const Vehicle& vehicle,
                              std::vector<JMTTrajectory1d>& trajectories);

  /**
   * @brief      Generate velocity keeping trajectory
   *
   * @param[in]  ego           The ego
   * @param      trajectories  The trajectories
   */
  void _GenerateVelocityKeepingTrajectory(const Ego& ego, std::vector<JMTTrajectory1d>& trajectories);

  /**
   * @brief      Generate Vehicle following trajectory
   *
   * @param[in]  ego           The ego
   * @param[in]  vehicle       The vehicle
   * @param      trajectories  The trajectories
   */
  void _GenerateVehicleFollowingTrajectory(const Ego& ego,
                                           const Vehicle& vehicle,
                                           std::vector<JMTTrajectory1d>& trajectories);

  /**
   * @brief      Generate stopping trajectory
   *
   * @param[in]  ego           The ego
   * @param      trajectories  The trajectories
   * @param      costs         The costs
   */
  void _GenerateStoppingTrajectory(const Ego& ego, std::vector<JMTTrajectory1d>& trajectories);

  Matrix32d _ComputeStartStatePy(const Vehicle& ego, const JMTTrajectory2d& prevTraj, const Waypoints& prevPath);

  JMTTrajectory2d _GenerataTrajectoryPy(const Ego& ego, const TrackedVehicleMap& trackedVehicleMap);

  const Configuration& _conf;
  const Map& _map;
  std::unique_ptr<JMTTrajectoryEvaluator> _pEvaluator;
};

}; // namespace pathplanning

#endif
