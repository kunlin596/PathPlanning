#ifndef PATHPLANNING_VEHICLE_H
#define PATHPLANNING_VEHICLE_H

#include <array>
#include <boost/format.hpp>

#include "map.h"
#include "math.h"
#include "perception.h"

namespace pathplanning {

struct VehicleConfiguration {
  VehicleConfiguration() {}
  VehicleConfiguration(double sPos, double sVel, double sAcc, double dPos,
                       double dVel, double dAcc)
      : sPos(sPos),
        sVel(sVel),
        sAcc(sAcc),
        dPos(dPos),
        dVel(dVel),
        dAcc(dAcc) {}

  double operator[](size_t index) {
    switch (index) {
      case 0:
        return sPos;
      case 1:
        return sVel;
      case 2:
        return sAcc;
      case 3:
        return dPos;
      case 4:
        return dVel;
      case 5:
        return dAcc;
      default:
        throw std::runtime_error("not valid index");
    }
  }

  double At(size_t index) const {
    switch (index) {
      case 0:
        return sPos;
      case 1:
        return sVel;
      case 2:
        return sAcc;
      case 3:
        return dPos;
      case 4:
        return dVel;
      case 5:
        return dAcc;
      default:
        throw std::runtime_error("not valid index");
    }
  }

  size_t Size() const { return 6; }

  friend VehicleConfiguration operator+(VehicleConfiguration lhs,
                                        const VehicleConfiguration &rhs) {
    // friends defined inside class body are inline and are hidden from non-ADL
    // lookup
    lhs.sPos += rhs.sPos;
    lhs.sVel += rhs.sVel;
    lhs.sAcc += rhs.sAcc;
    lhs.dPos += rhs.dPos;
    lhs.dVel += rhs.dVel;
    lhs.dAcc += rhs.dAcc;
    return lhs;
  }

  VehicleConfiguration &operator+=(const VehicleConfiguration &rhs) {
    sPos += rhs.sPos;
    sVel += rhs.sVel;
    sAcc += rhs.sAcc;
    dPos += rhs.dPos;
    dVel += rhs.dVel;
    dAcc += rhs.dAcc;
    return *this;
  }

  VehicleConfiguration &operator-=(const VehicleConfiguration &rhs) {
    sPos -= rhs.sPos;
    sVel -= rhs.sVel;
    sAcc -= rhs.sAcc;
    dPos -= rhs.dPos;
    dVel -= rhs.dVel;
    dAcc -= rhs.dAcc;
    return *this;
  }

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
  Vehicle(){};
  Vehicle(const int id, const VehicleConfiguration &conf)
      : _id(id), _conf(conf) {}
  virtual ~Vehicle() {}

  inline int GetId() const { return _id; }

  /**
   * @brief      Gets the predicted cofiguration `time` seconds from now.
   *
   * @param[in]  time  The time, in seconds
   *
   * @return     The cofiguration.
   */
  inline VehicleConfiguration GetConfiguration(const double time = 0.0) const {
    const double &sPos = _conf.sPos;
    const double &sVel = _conf.sVel;
    const double &sAcc = _conf.sAcc;
    const double &dPos = _conf.dPos;
    const double &dVel = _conf.dVel;
    const double &dAcc = _conf.dAcc;

    // clang-format off
    return {
        CalculatePosition(sPos, sVel, sAcc, time),
        CalculateVelocity(sVel, sAcc, time),
        sAcc,
        CalculatePosition(dPos, dVel, dAcc, time),
        CalculateVelocity(dVel, dAcc, time),
        dAcc
    };
    // clang-format on
  }

  void UpdateFromPerception(const Map::ConstPtr &pMap,
                            const Perception &perception);

  static Vehicle CreateFromPerception(const Map::ConstPtr &pMap,
                                      const Perception &perception);

 private:
  int _id = -1;
  VehicleConfiguration _conf;
};

using Vehicles = std::vector<Vehicle>;

}  // namespace pathplanning

// IO functions

inline std::ostream &operator<<(
    std::ostream &out, const pathplanning::VehicleConfiguration &conf) {
  out << (boost::format(
              "VehicleConfiguration=(sPos=%.3f, sVel=%.3f, sAcc=%.3f, "
              "dPos=%.3f, dVel=%.3f, dAcc=%.3f)") %
          conf.sPos % conf.sVel % conf.sAcc % conf.dPos % conf.dVel % conf.dAcc)
             .str();
  return out;
}

inline std::ostream &operator<<(std::ostream &out,
                                const pathplanning::Vehicle &vehicle) {
  out << "Vehicle(id=" << vehicle.GetId()
      << ", conf=" << vehicle.GetConfiguration() << ")";
  return out;
}

inline std::ostream &operator<<(std::ostream &out,
                                const pathplanning::Vehicles &vehicles) {
  using std::endl;
  out << std::string("{") << endl;
  for (const auto &v : vehicles) {
    out << std::string("  ") << v << endl;
  }
  out << std::string("}") << endl;
  return out;
}

#endif  // PATHPLANNING_VEHICLE_H
