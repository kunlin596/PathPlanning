#include "behaviorplanner.h"

namespace pathplanning {

std::vector<BehaviorState> BehaviorPlanner::GetSuccessorStates(
    const BehaviorState &state) const {
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

Vehicle BehaviorPlanner::GenerateProposal(
    const Vehicle &ego, const std::vector<BehaviorState> successorStates,
    const Predictions &predictions) const {
  // Dummy proposal to mvoe the car 30 meter ahead.
  return Vehicle(ego.GetId(), ego.GetConfiguration() +
                                  VehicleConfiguration(30.0, 0, 0, 0, 0, 0));
}

}  // namespace pathplanning
