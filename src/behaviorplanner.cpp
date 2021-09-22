#include "behaviorplanner.h"

namespace pathplanning {

BehaviorState GetNextBehavior(
  const BehaviorState &prevBehaviorState,
  const CarState &currCarState,
  const Path &prevPath,
  const ProbeData &probeData,
  const NaviMap &naviMap)
{
  constexpr double distanceToKeep = 30.0; // in meter
  constexpr double timeInterval = 0.02;
  constexpr double speedLimit = 49.5; // MPH

  double targetSpeed = 0.0;
  int targetLaneId = prevBehaviorState.laneId;


  const double currS = currCarState.frenetPose[0];
  const double currD = currCarState.frenetPose[1];

  for (size_t i = 0; i < probeData.size(); ++i) {
    const double speedX = probeData[i][3];
    const double speedY = probeData[i][4];
    const double speed = std::sqrt(speedX * speedX + speedY * speedY);
    const double s = probeData[i][5] + static_cast<double>(prevPath.size()) * timeInterval * speed;
    const double d = probeData[i][6];

    // If the the other car is in the same lane
    // TODO: Replace the range with continous d values to prevent accident
    if (GetDValueFromLandId(targetLaneId - 1) < (d - 2) and (d + 2) < GetDValueFromLandId(targetLaneId + 1)) {
      // std::cout << currCarState << std::endl;
      // std::cout << "  i="  << i << ", s=" << s << ", d=" << d << std::endl;
      if (s > currS and (s - currS) < distanceToKeep) {
        targetSpeed = std::min(speed, speedLimit); // speed limit
        if (prevBehaviorState.speed > targetSpeed) {
          targetSpeed -= 0.224;
        }
        auto state = BehaviorState(prevBehaviorState.laneId, targetSpeed);
        BOOST_LOG_TRIVIAL(debug) << "Detected car in the front, the distance is less than "
          << distanceToKeep << "(m), state=" << state;
        return state;
      }
    }
  }

  //
  // Start of behavior planning
  //

  // bool too_close = false;

  // // find ref_v to use
  // for (int i = 0; i < sensor_fusion.size(); ++i) {
  //   const auto &other_car = sensor_fusion[i];
  //   float d = other_car[6];
  //   // Check if the car is in our lane
  //   if ((2 + 4 * lane - 2) < d and d < (2 + 4 * lane + 2)) {
  //     double vx = other_car[3];
  //     double vy = other_car[4];
  //     double check_speed = sqrt(vx * vx + vy * vy);
  //     double check_car_s = other_car[5];

  //     // If using previous points can project s value out
  //     // check s values greater than mine and s gap
  //     //
  //     // Compensate the fact that we are re-using some trajectory points from the previous trejectory,
  //     // such that the current position of the
  //     // NOTE: Why do you need this? even through you are using the previous path points,
  //     // but every single time you are re-planning using the previous points as if it's started from scratch.
  //     // Because physically the car is not there yet. ??
  //     check_car_s += static_cast<double>(prev_size) * 0.02 * check_speed; // 0.02 is seconds

  //     if ((check_car_s > car_s) and (check_car_s - car_s) < 30) {
  //       // add logic here, lower reference velocity so we don't creash into the car in front of us,
  //       // could also flag so try to change lanes
  //       // ref_vel = 29.5 / 2.24;
  //       too_close = true;
  //       if (lane > 0) {
  //         lane = 0;
  //       }
  //     }
  //   }
  // }

  // if (too_close or ref_vel > 49.5 / 2.24) {
  //   ref_vel -= 0.224;
  // }
  // else if (ref_vel < 49.5) {
  //   ref_vel += 0.224;
  // }

  return BehaviorState(prevBehaviorState.laneId, std::min(currCarState.speed + 10.0, speedLimit));
}

}
