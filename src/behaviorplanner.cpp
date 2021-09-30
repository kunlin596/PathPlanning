#include "behaviorplanner.h"

namespace pathplanning {

BehaviorState GetNextBehavior(
  const BehaviorState &prevBehaviorState,
  const CarState &currCarState,
  const Path &prevPath,
  const ProbeData &probeData,
  const NaviMap &naviMap,
  const std::array<double, 2> &endPathFrenetPose)
{
  constexpr double distanceToKeep = 30; // in meter
  constexpr double timeInterval = 0.02;
  constexpr double speedLimit = 49.5; // MPH

  // Target speed is based on the previous behavior speed, not the current car speed.
  double targetSpeed = prevBehaviorState.speed;
  int targetLaneId = prevBehaviorState.laneId;

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

  for (size_t i = 0; i < probeData.size(); ++i) {
    // Sensed info for each car
    const double &speedX = probeData[i][3];
    const double &speedY = probeData[i][4];
    const double &speed = std::sqrt(speedX * speedX + speedY * speedY);
    // Predict where the car will be in the future
    const double s = probeData[i][5] + static_cast<double>(prevPath.size()) * timeInterval * speed / 2.24;
    const double &d = probeData[i][6];
    // If the the other car is in the same lane
    // TODO: Replace the range with continous d values to prevent accident
    if ((GetDValueFromLandId(targetLaneId - 1) + 2 < d) and
        d < (GetDValueFromLandId(targetLaneId + 1) - 2)) {

      double distance = s - expectedS;

      if (distance > 0 and distance < 10) {
        targetSpeed = speed;
        auto state = BehaviorState(prevBehaviorState.laneId, targetSpeed);
        return state;
      }

      if (distance > 0 and distance < distanceToKeep) {
        BOOST_LOG_TRIVIAL(debug) << "Detected car in the front, the distance is less than "
          << distanceToKeep << "(m)";
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
  auto state = BehaviorState(prevBehaviorState.laneId, targetSpeed);
  BOOST_LOG_TRIVIAL(debug) << state;
  return state;
}

}
