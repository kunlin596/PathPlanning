#ifndef BEHAVIORPLANNER_H
#define BEHAVIORPLANNER_H

#include "helpers.h"

namespace pathplanning {

class BehaviorPlanner {
public:

  BehaviorPlanner(const NaviMap &map)
    : _map(map) {}

  /**
   * Get successor states from given state
   */
  static std::vector<BehaviorState>
    GetSuccessorStates(const BehaviorState &state, int laneId);

  void Reset() {};

  void SetBehavior(const Behavior &behavior) { _behavior = behavior; }
  const Behavior& GetBehavior() const { return _behavior; }

private:
  Behavior _behavior;
  const NaviMap &_map;
};

} // end of pathplanning

#endif
