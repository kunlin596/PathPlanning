#ifndef BEHAVIORPLANNER_H
#define BEHAVIORPLANNER_H

#include "helpers.h"

namespace pathplanning {


class BehaviorPlanner {
public:
  BehaviorPlanner() {};
  ~BehaviorPlanner() {};

  BehaviorState GetNextBehavior(
    const BehaviorState &prevBehaviorState,
    const CarState &currCarState,
    const Path &prevPath,
    const ProbeData &probeData,
    const NaviMap &naviMap,
    const std::array<double, 2> &endPathFrenetPose);
};
} // end of pathplanning

#endif
