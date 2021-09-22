#ifndef BEHAVIORPLANNER_H
#define BEHAVIORPLANNER_H

#include "common.h"

namespace pathplanning {

BehaviorState GetNextBehavior(
  const BehaviorState &prevBehaviorState,
  const CarState &currCarState,
  const Path &prevPath,
  const ProbeData &probeData,
  const NaviMap &naviMap);

}

#endif
