#ifndef PATHPLANNING_VEHICLE_H
#define PATHPLANNING_VEHICLE_H

#include <array>

namespace pathplanning {

struct VehicleKinParams {
  VehicleKinParams() {}
  VehicleKinParams(double sPos, double sVel, double sAcc, double dPos,
                   double dVel, double dAcc)
      : sPos(sPos),
        sVel(sVel),
        sAcc(sAcc),
        dPos(dPos),
        dVel(dVel),
        dAcc(dAcc) {}
  double sPos = 0.0;
  double sVel = 0.0;
  double sAcc = 0.0;
  double dPos = 0.0;
  double dVel = 0.0;
  double dAcc = 0.0;
};

/**
 * @brief      This class describes a non-ego vehicle.
 */
class Vehicle {
 public:
  Vehicle(const VehicleKinParams &params) : _params(params) {}

  inline VehicleKinParams GetKinematicsByTime(const double time) {
    const double &sPos = _params.sPos;
    const double &sVel = _params.sVel;
    const double &sAcc = _params.sAcc;
    const double &dPos = _params.dPos;
    const double &dVel = _params.dVel;
    const double &dAcc = _params.dAcc;

    // clang-format off
    return VehicleKinParams(
        sPos + sVel * time + sAcc * time * time / 2.0,
        sVel + sAcc * time,
        sAcc,
        dPos + dVel * time + dAcc * time * time / 2.0,
        dVel + dAcc * time,
        dAcc
    );
    // clang-format on
  }

 private:
  VehicleKinParams _params;
};

}  // namespace pathplanning

#endif  // PATHPLANNING_VEHICLE_H
