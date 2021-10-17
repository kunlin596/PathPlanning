#include "behaviorplanner.h"

namespace pathplanning {

std::vector<BehaviorState> BehaviorPlanner::GetSuccessorStates(
    const BehaviorState &state) {
  // FIXME: Remove hard coded lane id check
  std::vector<BehaviorState> states = {BehaviorState::kLaneKeeping};
  if (state == BehaviorState::kLaneKeeping) {
    states.push_back(BehaviorState::kLeftLaneChangePreparation);
    states.push_back(BehaviorState::kRightLaneChangePreparation);
  } else if (state == BehaviorState::kLeftLaneChangePreparation) {
    states.push_back(BehaviorState::kLeftLaneChangePreparation);
    states.push_back(BehaviorState::kLeftLaneChange);
  } else if (state == BehaviorState::kRightLaneChangePreparation) {
    states.push_back(BehaviorState::kRightLaneChangePreparation);
    states.push_back(BehaviorState::kRightLaneChange);
  }
  return states;
}

}  // namespace pathplanning
