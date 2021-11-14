#pragma once

#include <array>

#include "Eigen-3.3/Eigen/Dense"
#include "configuration.h"
#include "json.hpp"
#include "log.h"
#include "map.h"
#include "math.h"

namespace Eigen {
using Vector6d = Matrix<double, 6, 1>;
using Matrix6d = Matrix<double, 6, 6>;
using Matrix62d = Matrix<double, 6, 2>;
using Matrix32d = Matrix<double, 3, 2>;
};

namespace pathplanning {

using Eigen::Matrix32d;
using Eigen::Matrix62d;
using Eigen::Matrix6d;
using Eigen::Vector6d;

using namespace nlohmann;

/**
 * @brief      This class describes a vehicle.
 */
struct Vehicle
{
  static constexpr double Size = 4.5;

  Vehicle() {}

  Vehicle(int id)
    : _id(id)
  {}

  Vehicle(const Matrix32d& kinematics)
    : _kinematics(kinematics)
  {}

  Vehicle(const int id, const Matrix32d& kinematics)
    : _id(id)
    , _kinematics(kinematics)
  {}

  virtual ~Vehicle() {}

  /**
   * @brief      Gets the predicted cofiguration `time` seconds from now.
   *
   * @param[in]  time  The time, in seconds
   *
   * @return     The cofiguration.
   */

  bool IsEgo() const { return _id == std::numeric_limits<int>::max(); }

  Matrix32d GetKinematics(double time) const;

  friend std::ostream& operator<<(std::ostream& out, const Vehicle& vehicle);

  static const Eigen::Matrix3d GetMotionModel(double time);

  json Dump() const;

  int _id = -1;
  Matrix32d _kinematics = Matrix32d::Zero();
};

/**
 * @brief      A thin wrapper data class for ego vehicle
 *
 * Velocity and accelatation of s and d will be computed when ego collects 3 new
 * localization (measurements) from simulator. They could be calculated using
 * simple kinematics equations.
 */
struct Ego : public Vehicle
{
  Ego() {}

  virtual ~Ego() {}

  /**
   * @brief      Update new localization (measurements) from simulator
   *
   * @param[in]  x      New x
   * @param[in]  y      New y
   * @param[in]  s      New s
   * @param[in]  d      New d
   * @param[in]  yaw    The yaw angle in degrees
   * @param[in]  speed  The speed in meters/second
   */
  void
  Update(double x, double y, double s, double d, double yaw, double speed, const Map& map, const Configuration& conf);

  json Dump() const;

  double _x = 0.0;
  double _y = 0.0;
  double _yaw = 0.0;
  double _speed = 0.0;
};

using Vehicles = std::vector<Vehicle>;

std::ostream&
operator<<(std::ostream& out, const pathplanning::Vehicle& vehicle);

std::ostream&
operator<<(std::ostream& out, const pathplanning::Vehicles& vehicles);

} // namespace pathplanning

// IO functions
