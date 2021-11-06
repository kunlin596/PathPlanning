#ifndef PATHPLANNING_PTG_H
#define PATHPLANNING_PTG_H

#include "jmt.h"
#include "json.hpp"
#include "tracker.h"
#include "traj_evaluator.h"

namespace pathplanning {

using namespace nlohmann;

/**
 * @brief      This class describes a polynomial trajectory generator.
 */
class PolynomialTrajectoryGenerator
{
public:
  explicit PolynomialTrajectoryGenerator(const Map& map, const Configuration& conf);
  virtual ~PolynomialTrajectoryGenerator() {}

  /**
   * @brief      Generate drivable trajectory (a set of waypoints)
   *
   * PTG uses start and end configurations of from the behavior planner to
   * generate trajectories. `targetVehicleId` is the id of the target vehicle
   * that we are setting out trajectory relative to.
   *
   * `deltaConf` is the configuration offset w.r.t. the target vehicle.
   *
   * `targetVehicleId` is the id of the targeting vehicle in the perception
   * result list. When `-1` is provided, it means we don't want to plan w.r.t.
   * any vehicle, the trajectory will fallback to lane keeping.
   *
   * So if at time 5 the target vehicle will be at [100, 10, 0, 0, 0, 0] and
   * delta is [-10, 0, 0, 4, 0, 0], then our goal state for t = 5 will be [90,
   * 10, 0, 4, 0, 0]. This would correspond to a goal of "follow 10 meters
   * behind and 4 meters to the right of target vehicle".
   *
   * `t` is the desired time at which we will be at the goal (relative to now as
   * t=0).
   *
   * `perceptions` is a map of {id: Vehicle}. The vehicle represents the current
   * state of the vehicle, and can be used to predict future states.
   *
   */
  std::pair<Waypoints, JMTTrajectory2d> GeneratePath(const JMTTrajectory2d& proposal,
                                                     const TrackedVehicleMap& trackedVehicleMap,
                                                     json& log);

  Matrix32d ComputeStartState(const Vehicle& ego, const JMTTrajectory2d& prevTraj, const Waypoints& prevPath);

private:
  const Configuration& _conf;
  const Map& _map;
  std::unique_ptr<JMTTrajectoryEvaluator> _pEvaluator;
};

}; // namespace pathplanning

#endif
