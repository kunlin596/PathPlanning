#ifndef BEHAVIORPLANNER_H
#define BEHAVIORPLANNER_H

#include "helpers.h"

namespace pathplanning {

class BehaviorPlanner {
public:

  static std::vector<BehaviorState> GetNextBehavior(
    const BehaviorState &prevBehaviorState,
    const CarState &currCarState,
    const Path &prevPath,
    const SensorFusions &sensorFusions,
    const NaviMap &naviMap,
    const std::array<double, 2> &endPathFrenetPose);

  /**
   * Get successor states from given state
   */
  static std::vector<BehaviorState>
    GetSuccessorStates(const BehaviorState &state, int laneId);
};

} // end of pathplanning

#endif
