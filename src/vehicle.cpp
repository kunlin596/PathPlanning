#include "vehicle.h"

#include "configuration.h"

namespace pathplanning {

double
VehicleConfiguration::operator[](size_t index)
{
  return At(index);
}

double
VehicleConfiguration::At(size_t index) const
{
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
      throw std::runtime_error("Invalid index");
  }
}

VehicleConfiguration
operator+(VehicleConfiguration lhs, const VehicleConfiguration& rhs)
{
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

VehicleConfiguration
operator-(VehicleConfiguration lhs, const VehicleConfiguration& rhs)
{
  // friends defined inside class body are inline and are hidden from non-ADL
  // lookup
  lhs.sPos -= rhs.sPos;
  lhs.sVel -= rhs.sVel;
  lhs.sAcc -= rhs.sAcc;
  lhs.dPos -= rhs.dPos;
  lhs.dVel -= rhs.dVel;
  lhs.dAcc -= rhs.dAcc;
  return lhs;
}

VehicleConfiguration&
VehicleConfiguration::operator+=(const VehicleConfiguration& rhs)
{
  sPos += rhs.sPos;
  sVel += rhs.sVel;
  sAcc += rhs.sAcc;
  dPos += rhs.dPos;
  dVel += rhs.dVel;
  dAcc += rhs.dAcc;
  return *this;
}

VehicleConfiguration&
VehicleConfiguration::operator-=(const VehicleConfiguration& rhs)
{
  sPos -= rhs.sPos;
  sVel -= rhs.sVel;
  sAcc -= rhs.sAcc;
  dPos -= rhs.dPos;
  dVel -= rhs.dVel;
  dAcc -= rhs.dAcc;
  return *this;
}

void
KinematicsTracker::Update(double pos)
{
  _measursments.push_back(pos);

  if (_measursments.size() > _numMeasurementsToTrack) {
    _measursments.erase(_measursments.begin());
  }

  if (_measursments.size() > 2) {
    size_t index3 = _measursments.size() - 1;
    size_t index2 = _measursments.size() - 2;
    size_t index1 = _measursments.size() - 3;
    double dist1 = _measursments[index2] - _measursments[index1];
    double dist2 = _measursments[index3] - _measursments[index1];
    double t1 = _timeStep;
    double t2 = t1 * t1;
    Eigen::Matrix2d A;

    // clang-format off
    A <<
      t1, 0.5 * t2,
      t2, 0.5 * t2;
    // clang-format on

    Eigen::Vector2d b;
    b << dist1, dist2;
    Eigen::Vector2d x = A.inverse() * b;
    _values[0] = pos;
    _values[1] = x[0];
    _values[2] = x[1];
  }
}

void
Vehicle::Update(const Perception& perception)
{
  const auto& sd = perception.sd;
  _sTracker.Update(sd[0]);
  _dTracker.Update(sd[1]);
  auto sValues = _sTracker.GetValues();
  auto dValues = _dTracker.GetValues();
  _conf = VehicleConfiguration(
    sValues[0], sValues[1], sValues[2], dValues[0], dValues[1], dValues[2]);
}

Vehicle
Vehicle::CreateFromPerception(const Map::ConstPtr& pMap,
                              const Perception& perception,
                              int numMeasurementsToTrack,
                              double timeStep)
{
  VehicleConfiguration conf;
  conf.sPos = perception.sd[0];
  conf.dPos = perception.sd[1];

  std::array<double, 2> sd2 =
    pMap->GetSD(perception.xy[0] + perception.vel[0],
                perception.xy[1] + perception.vel[1],
                std::atan2(perception.vel[1], perception.vel[0]));

  conf.sVel = (sd2[0] - conf.sPos);
  conf.dVel = (sd2[1] - conf.dPos);
  // Assume constant accelaration

  return Vehicle(perception.id, conf, numMeasurementsToTrack, timeStep);
}

VehicleConfiguration
VehicleConfiguration::GetConfiguration(const double time) const
{
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

VehicleConfiguration
Vehicle::GetConfiguration(const double time) const
{
  return _conf.GetConfiguration(time);
}

Ego::Ego(double x, double y, double s, double d, double yaw, double speed)
  : _x(x)
  , _y(y)
  , _yaw(yaw)
  , _speed(speed)
{
  Update(x, y, s, d, yaw, speed);
}

void
Ego::Update(double x, double y, double s, double d, double yaw, double speed)
{
  _x = x;
  _y = y;
  _yaw = yaw;
  _speed = speed;

  _sTracker.Update(s);
  _dTracker.Update(d);
  auto sValues = _sTracker.GetValues();
  auto dValues = _dTracker.GetValues();
  _conf = VehicleConfiguration(
    sValues[0], sValues[1], sValues[2], dValues[0], dValues[1], dValues[2]);
}

std::ostream&
operator<<(std::ostream& out, const pathplanning::VehicleConfiguration& conf)
{
  return out << fmt::format("VehicleConfiguration("
                            "sPos={:7.3f}, sVel={:7.3f}, sAcc={:7.3f}, "
                            "dPos={:7.3f}, dVel={:7.3f}, dAcc={:7.3f})",
                            conf.sPos,
                            conf.sVel,
                            conf.sAcc,
                            conf.dPos,
                            conf.dVel,
                            conf.dAcc);
}

std::ostream&
operator<<(std::ostream& out, const pathplanning::Vehicle& vehicle)
{
  return out << fmt::format("Vehicle(id={:2d}, conf={:s}",
                            vehicle.GetId(),
                            vehicle.GetConfiguration());
}

std::ostream&
operator<<(std::ostream& out, const pathplanning::Vehicles& vehicles)
{
  out << std::string("{\n");
  for (const auto& v : vehicles) {
    out << fmt::format("  {:s}\n", v);
  }
  return out << std::string("}\n");
}

} // namespace pathplanning
