#ifndef PATHPLANNING_VEHICLE_H
#define PATHPLANNING_VEHICLE_H

#include <array>

namespace pathplanning {

/**
 * @brief      This class describes a non-ego vehicle.
 */
class Vehicle {
 public:
  Vehicle(const std::array<double, 6> &startState) : _startState(startState) {}

  inline std::array<double, 6> GetStateByTime(const double time) {
    const double &sPos = _startState[0];
    const double &sVel = _startState[1];
    const double &sAcc = _startState[2];
    const double &dPos = _startState[4];
    const double &dVel = _startState[5];
    const double &dAcc = _startState[6];

    // clang-format off
    return {
        sPos + sVel * time + sAcc * time * time / 2.0,
        sVel + sAcc * time,
        sAcc,
        dPos + dVel * time + dAcc * time * time / 2.0,
        dVel + dAcc * time,
        dAcc,
    };
    // clang-format on
  }

 private:
  std::array<double, 6> _startState;
};

}  // namespace pathplanning

#endif  // PATHPLANNING_VEHICLE_H
