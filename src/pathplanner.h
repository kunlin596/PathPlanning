#ifndef PLANNER_H
#define PLANNER_H

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/Dense"
#include "Eigen-3.3/Eigen/QR"
#include "common.h"

namespace pathplanning {

///
/// Path Generators
///

/**
 * Based on the given behavior, generate the goal for path planner to plan.
 *
 * To generate the goal, the goal generator needs to know the ego motion and non-ego motions on the road,
 * since it has to adjust the target speed and target lane for the planner.
 */
class GoalGenerator {
public:
  explicit GoalGenerator(const NaviMap &map)
    : _map(map) {}

  Goal GenerateGoal(
    const BehaviorState &behaviorState,
    const double speedReference,
    const SensorFusions &sensorFusions,
    const CarState &carState,
    const Path &prevPath
  );

private :
  const NaviMap &_map;

  Goal _GenerateLKGoal(
    const double speedReference,
    const SensorFusions &sensorFusions,
    const CarState &carState,
    const Path &prevPath);

  Goal _GenerateLCPGoal(
    int direction, 
    const double speedReference,
    const SensorFusions &sensorFusions,
    const CarState &carState,
    const Path &prevPath);

  Goal _GenerateLCGoal(
    int direction, 
    const double speedReference,
    const SensorFusions &sensorFusions,
    const CarState &carState,
    const Path &prevPath);
};

class PathPlanner {
public:

  PathPlanner(const NaviMap &map)
    : _map(map) {}

  /**
   * Generate path using spline line fitting
   */
  Path GeneratePath(const Goal &goal, const Behavior &currentBehavior);

  double EvaluatePath(
    const Goal &goal, const Path &path, const std::vector<SensorFusions> &laneSensorFusions,
    const std::array<double, 3> &averSpeeds);


  void UpdateCarState(const CarState &carState) { _carState = carState; }
  void UpdatePathCache(const Path &path) { _prevPath = path; }

private:

  /**
   * Filter sensed car locations within radius
   * @param  sensorFusions [description]
   * @param  radius        [description]
   * @return               [description]
   */
  SensorFusions _Filter(const SensorFusions &sensorFusions, double radius);

  float _ComputeSpeedScore(const Goal goal, const std::array<double, 3> &averSpeeds)
  {
    auto maxAverSpeed = std::max_element(averSpeeds.begin(), averSpeeds.end());
    int maxIndex = std::distance(averSpeeds.begin(), maxAverSpeed);

    if (goal.laneId == maxIndex) {
      return 1.0;
    }
    return 0.0;
  }

  Path _GeneratePath(
    const double targetDValue,
    const double targetSValue,
    const double speedReference);

  const NaviMap &_map; ///< Navigation map
  CarState _carState; ///< Current car state
  Path _prevPath; ///< Cached planned paths

};


} // end of pathplanning
#endif
