#include "behaviorplanner.h"
#include "common.h"

namespace pathplanning {


std::vector<BehaviorState>
BehaviorPlanner::GetSuccessorStates(const BehaviorState &state, int laneId)
{
  // FIXME: Remove hard coded lane id check
  std::vector<BehaviorState> states = { BehaviorState::kLaneKeeping };
  if (state == BehaviorState::kLaneKeeping) {
    states.push_back(BehaviorState::kLeftLaneChangePreparation);
    states.push_back(BehaviorState::kRightLaneChangePreparation);
  } else if (state == BehaviorState::kLeftLaneChangePreparation) {
    if (laneId - 1 > 0) {
      states.push_back(BehaviorState::kLeftLaneChangePreparation);
      states.push_back(BehaviorState::kLeftLaneChange);
    }
  } else if (state == BehaviorState::kRightLaneChangePreparation) {
    if (laneId + 1 <= 2) {
      states.push_back(BehaviorState::kRightLaneChangePreparation);
      states.push_back(BehaviorState::kRightLaneChange);
    }
  }
  return states;
}

} // end of pathplanning
