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
    const Vehicle &ego, const Waypoints &prevPath,
    const Waypoint &endPrevPathSD,
    const std::vector<BehaviorState> successorStates,
    const Predictions &predictions) const {
  // Find out the vehicle in front of ego
  // leftVehicles
  for (const auto &prediction : predictions) {
    int id = prediction.first;
    const auto &predictedVehicleStates = prediction.second;
  }

  const auto &conf = ego.GetConfiguration();
  return Vehicle(
      ego.GetId(),
      VehicleConfiguration(conf.sPos + 30.0, mph2ms(Configuration::SPEED_LIMIT),
                           0.0, conf.dPos, 0.0, 0.0));
}

}  // namespace pathplanning
