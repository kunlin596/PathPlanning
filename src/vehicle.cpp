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
Vehicle::Update(const Perception& perception)
{
  assert(_id == perception.id);
  const auto& sd = perception.sd;
  _conf = VehicleConfiguration(perception.sd[0],
                               perception.sdVel[0],
                               0.0,
                               perception.sd[1],
                               perception.sdVel[1],
                               0.0);
}

Vehicle
Vehicle::CreateFromPerception(const Map::ConstPtr& pMap,
                              const Perception& perception,
                              double timeStep)
{
  VehicleConfiguration conf;
  auto sdVel = ComputeSDVelocity(
    pMap, perception.xy, perception.xyVel, perception.sd, timeStep);
  conf.sVel = sdVel[0];
  conf.dVel = sdVel[1];
  // Assume constant accelaration, do nothing

  return Vehicle(perception.id, conf);
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

  // TODO: Compute sd velocity
  _conf = VehicleConfiguration(s, speed, 0.0, d, 0.0, 0.0);
}

std::ostream&
operator<<(std::ostream& out, const pathplanning::VehicleConfiguration& conf)
{
  return out << fmt::format("VehicleConf("
                            "sPos={:5.3f}, sVel={:3.3f}, sAcc={:3.3f}, "
                            "dPos={:5.3f}, dVel={:3.3f}, dAcc={:3.3f})",
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
