#include "behaviorplanner.h"

#include <cmath>

namespace pathplanning {

double CostFunctions::GetGoalDistanceCost(int goalLaneId, int intendedLaneId,
                                          int finalLaneId,
                                          double distanceToGoal) {
  // The cost increases with both the distance of intended lane from the goal
  //   and the distance of the final lane from the goal. The cost of being out
  //   of the goal lane also becomes larger as the vehicle approaches the
  //   goal.
  int deltaD = 2.0 * goalLaneId - intendedLaneId - finalLaneId;
  return 1.0 - std::exp(-(std::abs(deltaD) / distanceToGoal));
};

double CostFunctions::GetInefficiencyCost(int targetSpeed, int intendedLane,
                                          int finalLaneId,
                                          const std::vector<double> &laneSpeeds) {
  // Cost becomes higher for trajectories with intended lane and final lane
  //   that have traffic slower than targetSpeed.
  double intendedSpeed = laneSpeeds[intendedLane];
  double finalSpeed = laneSpeeds[finalLaneId];
  return (2.0 * targetSpeed - intendedSpeed - finalSpeed) / targetSpeed;
}

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
