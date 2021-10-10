#include "behaviorplanner.h"
#include "common.h"

namespace pathplanning {

// namespace cost {
// /**
//  * TODO: change weights for cost functions.
//  */
// const float REACH_GOAL = pow(10, 6);
// const float EFFICIENCY = pow(10, 5);
// 
// // Here we have provided two possible suggestions for cost functions, but feel 
// //   free to use your own! The weighted cost over all cost functions is computed
// //   in calculate_cost. The data from get_helper_data will be very useful in 
// //   your implementation of the cost functions below. Please see get_helper_data
// //   for details on how the helper data is computed.
// 
// float goal_distance_cost(
//     const Vehicle &vehicle, 
//     const vector<Vehicle> &trajectory, 
//     const map<int, vector<Vehicle>> &predictions, 
//     map<string, float> &data) {
//   // Cost increases based on distance of intended lane (for planning a lane 
//   //   change) and final lane of trajectory.
//   // Cost of being out of goal lane also becomes larger as vehicle approaches 
//   //   goal distance.
//   // This function is very similar to what you have already implemented in the 
//   //   "Implement a Cost Function in C++" quiz.
//   float cost;
//   float distance = data["distance_to_goal"];
//   if (distance > 0) {
//     cost = 1 - 2*exp(-(abs(2.0*vehicle.goal_lane - data["intended_lane"] 
//          - data["final_lane"]) / distance));
//   } else {
//     cost = 1;
//   }
// 
//   return cost;
// }
// 
// float inefficiency_cost(const Vehicle &vehicle, 
//                         const vector<Vehicle> &trajectory, 
//                         const map<int, vector<Vehicle>> &predictions, 
//                         map<string, float> &data) {
//   // Cost becomes higher for trajectories with intended lane and final lane 
//   //   that have traffic slower than vehicle's target speed.
//   // You can use the lane_speed function to determine the speed for a lane. 
//   // This function is very similar to what you have already implemented in 
//   //   the "Implement a Second Cost Function in C++" quiz.
//   float proposed_speed_intended = lane_speed(predictions, data["intended_lane"]);
//   if (proposed_speed_intended < 0) {
//     proposed_speed_intended = vehicle.target_speed;
//   }
// 
//   float proposed_speed_final = lane_speed(predictions, data["final_lane"]);
//   if (proposed_speed_final < 0) {
//     proposed_speed_final = vehicle.target_speed;
//   }
//     
//   float cost = (2.0*vehicle.target_speed - proposed_speed_intended 
//              - proposed_speed_final)/vehicle.target_speed;
// 
//   return cost;
// }
// 
// float lane_speed(const map<int, vector<Vehicle>> &predictions, int lane) {
//   // All non ego vehicles in a lane have the same speed, so to get the speed 
//   //   limit for a lane, we can just find one vehicle in that lane.
//   for (map<int, vector<Vehicle>>::const_iterator it = predictions.begin(); 
//        it != predictions.end(); ++it) {
//     int key = it->first;
//     Vehicle vehicle = it->second[0];
//     if (vehicle.lane == lane && key != -1) {
//       return vehicle.v;
//     }
//   }
//   // Found no vehicle in the lane
//   return -1.0;
// }
// 
// float calculate_cost(const Vehicle &vehicle, 
//                      const map<int, vector<Vehicle>> &predictions, 
//                      const vector<Vehicle> &trajectory) {
//   // Sum weighted cost functions to get total cost for trajectory.
//   map<string, float> trajectory_data = get_helper_data(vehicle, trajectory, 
//                                                        predictions);
//   float cost = 0.0;
// 
//   // Add additional cost functions here.
//   vector<std::function<float(const Vehicle &, const vector<Vehicle> &, 
//                              const map<int, vector<Vehicle>> &, 
//                              map<string, float> &)
//     >> cf_list = {goal_distance_cost, inefficiency_cost};
//   vector<float> weight_list = {REACH_GOAL, EFFICIENCY};
//     
//   for (int i = 0; i < cf_list.size(); ++i) {
//     float new_cost = weight_list[i]*cf_list[i](vehicle, trajectory, predictions, 
//                                                trajectory_data);
//     cost += new_cost;
//   }
// 
//   return cost;
// }
// } // end cost                                                                      



std::vector<BehaviorState> BehaviorPlanner::GetNextBehavior(
  const BehaviorState &prevBehaviorState,
  const CarState &currCarState,
  const Path &prevPath,
  const SensorFusions &sensorFusions,
  const NaviMap &naviMap,
  const std::array<double, 2> &endPathFrenetPose)
{
  constexpr double distanceToKeep = 30; // in meter
  constexpr double timeInterval = 0.02;
  constexpr double speedLimit = 49.5; // MPH

  // Target speed is based on the previous behavior speed, not the current car speed.
  double targetSpeed = speedLimit;
  int targetLaneId = 0;

  const double currS = currCarState.frenetPose[0];
  const double currD = currCarState.frenetPose[1];

  // Expected S value of the car if the previous trajectory are executed
  double expectedS = currS;

  if (prevPath.size() > 0) {
    expectedS = endPathFrenetPose[0];
  }

  double closestCarSpeed = std::numeric_limits<double>::quiet_NaN();
  double closestCarS = std::numeric_limits<double>::max();

  bool tooClose = false;

  for (size_t i = 0; i < sensorFusions.size(); ++i) {
    // Sensed info for each car
    const double &speed = sensorFusions[i].speed;
    // Predict where the car will be in the future
    const double s = sensorFusions[i].frenetPose[0] + static_cast<double>(prevPath.size()) * timeInterval * speed / 2.24;
    const double &d = sensorFusions[i].frenetPose[1];
    // If the the other car is in the same lane
    // TODO: Replace the range with continous d values to prevent accident
    if ((naviMap.road.GetLaneCenterDValue(targetLaneId - 1) < d) and
        d < (naviMap.road.GetLaneCenterDValue(targetLaneId + 1))) {

      double distance = s - expectedS;

      if (distance > 0 and distance < 10) {
        targetSpeed = speed;
        Behavior state = Behavior(0, targetSpeed);
        return {BehaviorState::kLaneKeeping};
      }

      if (distance > 0 and distance < distanceToKeep) {
        std::cout << (boost::format("Detected car in the front, the distance is less than %.3f (m)\n")
            % distanceToKeep).str();
        if (s < closestCarS) {
          closestCarS = s;
          closestCarSpeed = speed;
        }
      }
    }
  }

  constexpr double acc = 0.224;  // ~5 meters / second^2
  if (!std::isnan(closestCarSpeed)) {
    targetSpeed -= acc;
  } else if (targetSpeed < speedLimit) {
    targetSpeed += acc;
  }

  targetSpeed = std::min(targetSpeed, speedLimit); // Speed limit
  Behavior state(targetLaneId, targetSpeed);
  std::cout << state << std::endl;
  return {BehaviorState::kLaneKeeping};
}


std::vector<BehaviorState>
BehaviorPlanner::GetSuccessorStates(const BehaviorState &state, int laneId)
{
  // FIXME: Remove hard coded lane id check
  std::vector<BehaviorState> states = {BehaviorState::kLaneKeeping};
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
