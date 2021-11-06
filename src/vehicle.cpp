#include "vehicle.h"
#include "configuration.h"

namespace pathplanning {

const Eigen::Matrix3d
Vehicle::GetMotionModel(double time)
{
  Eigen::Matrix3d motionModel = Eigen::Matrix3d::Identity();
  motionModel(0, 1) = time;
  motionModel(0, 2) = 0.5 * time * time;
  motionModel(1, 2) = time;
  return motionModel;
}

Matrix32d
Vehicle::GetKinematics(double time) const
{
  if (time > 1e-6) {
    return GetMotionModel(time) * _kinematics;
  }
  return _kinematics;
}

json
Vehicle::Dump() const
{
  json j = json::array();
  // s
  j.push_back(_kinematics(0, 0));
  j.push_back(_kinematics(1, 0));
  j.push_back(_kinematics(2, 0));
  // d
  j.push_back(_kinematics(0, 1));
  j.push_back(_kinematics(1, 1));
  j.push_back(_kinematics(2, 1));
  return { { "id", _id }, { "kinematics", j } };
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
  _kinematics.col(0) << s, speed, 0.0;
  _kinematics.col(1) << d, 0.0, 0.0;
}

json
Ego::Dump() const
{
  json j = Vehicle::Dump();
  j["yaw"] = _yaw;
  j["speed"] = _speed;
  return j;
}

std::ostream&
operator<<(std::ostream& out, const pathplanning::Vehicle& vehicle)
{
  return out << fmt::format("id={:2d}, kinematics=(s: {}, d: {})",
                            vehicle._id,
                            vehicle._kinematics.col(0).transpose().format(HeavyFmt),
                            vehicle._kinematics.col(1).transpose().format(HeavyFmt));
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
