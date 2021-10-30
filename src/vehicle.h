#ifndef PATHPLANNING_VEHICLE_H
#define PATHPLANNING_VEHICLE_H

#include <array>

#include "Eigen-3.3/Eigen/Dense"
#include "configuration.h"
#include "log.h"
#include "map.h"
#include "math.h"
#include "perception.h"

namespace pathplanning {

struct VehicleConfiguration
{
  VehicleConfiguration() {}
  VehicleConfiguration(double sPos,
                       double sVel,
                       double sAcc,
                       double dPos,
                       double dVel,
                       double dAcc)
    : sPos(sPos)
    , sVel(sVel)
    , sAcc(sAcc)
    , dPos(dPos)
    , dVel(dVel)
    , dAcc(dAcc)
  {}
  VehicleConfiguration(const std::array<double, 6>& params)
    : sPos(params[0])
    , sVel(params[1])
    , sAcc(params[2])
    , dPos(params[3])
    , dVel(params[4])
    , dAcc(params[5])
  {}

  // VehicleConfiguration(const VehicleConfiguration &other)
  //     : sPos(other.sPos),
  //       sVel(other.sVel),
  //       sAcc(other.sAcc),
  //       dPos(other.dPos),
  //       dVel(other.dVel),
  //       dAcc(other.dAcc) {}

  // VehicleConfiguration(VehicleConfiguration &&other)
  //     : sPos(std::move(other.sPos)),
  //       sVel(std::move(other.sVel)),
  //       sAcc(std::move(other.sAcc)),
  //       dPos(std::move(other.dPos)),
  //       dVel(std::move(other.dVel)),
  //       dAcc(std::move(other.dAcc)) {}

  size_t Size() const { return 6; }

  double operator[](size_t index);

  double At(size_t index) const;

  friend VehicleConfiguration operator+(VehicleConfiguration lhs,
                                        const VehicleConfiguration& rhs);

  friend VehicleConfiguration operator-(VehicleConfiguration lhs,
                                        const VehicleConfiguration& rhs);

  VehicleConfiguration& operator+=(const VehicleConfiguration& rhs);

  VehicleConfiguration& operator-=(const VehicleConfiguration& rhs);

  VehicleConfiguration GetConfiguration(const double time = 0.0) const;

  friend std::ostream& operator<<(std::ostream& out,
                                  const VehicleConfiguration& vehicle);

  double sPos = 0.0;
  double sVel = 0.0;
  double sAcc = 0.0;
  double dPos = 0.0;
  double dVel = 0.0;
  double dAcc = 0.0;
};

VehicleConfiguration
operator+(VehicleConfiguration lhs, const VehicleConfiguration& rhs);

/**
 * @brief      This class describes a kinematics calculator.
 *
 * It collects position measurements and use it to compute velocity and
 * accelaration .
 */
class KinematicsTracker
{
public:
  KinematicsTracker() {}
  KinematicsTracker(int numMeasurementsToTrack, double timeStep)
    : _numMeasurementsToTrack(numMeasurementsToTrack)
    , _timeStep(timeStep)
  {}
  std::array<double, 3> GetValues() const { return _values; }

  void Update(double pos);

private:
  int _numMeasurementsToTrack = 30;
  double _timeStep = 0.02;
  std::vector<double>
    _measursments; ///< For calculating speed and accelaration.
  std::array<double, 3> _values = { 0.0, 0.0, 0.0 };
};

/**
 * @brief      This class describes a vehicle.
 */
class Vehicle
{
public:
  Vehicle(){};
  Vehicle(const int id,
          const VehicleConfiguration& conf,
          int numMeasurementsToTrack,
          double timeStep)
    : _id(id)
    , _conf(conf)
    , _sTracker(KinematicsTracker(numMeasurementsToTrack, timeStep))
    , _dTracker(KinematicsTracker(numMeasurementsToTrack, timeStep))
  {}
  virtual ~Vehicle() {}

  inline int GetId() const { return _id; }

  /**
   * @brief      Gets the predicted cofiguration `time` seconds from now.
   *
   * @param[in]  time  The time, in seconds
   *
   * @return     The cofiguration.
   */
  VehicleConfiguration GetConfiguration(const double time = 0.0) const;

  bool IsEgo() const { return _id == std::numeric_limits<int>::max(); }

  void Update(const Perception& perception);

  static Vehicle CreateFromPerception(const Map::ConstPtr& pMap,
                                      const Perception& perception,
                                      int numMeasurementsToTrack,
                                      double timeStep);

  friend std::ostream& operator<<(std::ostream& out, const Vehicle& vehicle);

protected:
  int _id = -1;
  VehicleConfiguration _conf;

  KinematicsTracker _sTracker; ///< For estimating s kinematics
  KinematicsTracker _dTracker; ///< For estimating d kinematics
};

/**
 * @brief      A thin wrapper data class for ego vehicle
 *
 * Velocity and accelatation of s and d will be computed when ego collects 3 new
 * localization (measurements) from simulator. They could be calculated using
 * simple kinematics equations.
 */
class Ego : public Vehicle
{
public:
  Ego() {}
  Ego(double x, double y, double s, double d, double yaw, double speed);

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
  void Update(double x, double y, double s, double d, double yaw, double speed);

private:
  double _x = 0.0;
  double _y = 0.0;
  double _yaw = 0.0;
  double _speed = 0.0;
};

using Vehicles = std::vector<Vehicle>;

std::ostream&
operator<<(std::ostream& out, const pathplanning::VehicleConfiguration& conf);

std::ostream&
operator<<(std::ostream& out, const pathplanning::Vehicle& vehicle);

std::ostream&
operator<<(std::ostream& out, const pathplanning::Vehicles& vehicles);

} // namespace pathplanning

// IO functions

#endif // PATHPLANNING_VEHICLE_H
