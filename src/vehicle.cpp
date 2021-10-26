#include "vehicle.h"

#include "configuration.h"

namespace pathplanning {

double VehicleConfiguration::operator[](size_t index) { return At(index); }

double VehicleConfiguration::At(size_t index) const {
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

VehicleConfiguration operator+(VehicleConfiguration lhs,
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

VehicleConfiguration operator-(VehicleConfiguration lhs,
                               const VehicleConfiguration &rhs) {
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

VehicleConfiguration &VehicleConfiguration::operator+=(
    const VehicleConfiguration &rhs) {
  sPos += rhs.sPos;
  sVel += rhs.sVel;
  sAcc += rhs.sAcc;
  dPos += rhs.dPos;
  dVel += rhs.dVel;
  dAcc += rhs.dAcc;
  return *this;
}

VehicleConfiguration &VehicleConfiguration::operator-=(
    const VehicleConfiguration &rhs) {
  sPos -= rhs.sPos;
  sVel -= rhs.sVel;
  sAcc -= rhs.sAcc;
  dPos -= rhs.dPos;
  dVel -= rhs.dVel;
  dAcc -= rhs.dAcc;
  return *this;
}

Ego::Ego(double x, double y, double s, double d, double yaw, double speed)
    : x(x), y(y), s(s), d(d), yaw(yaw), speed(speed) {
  UpdateObvervation(s, d);
}

void Ego::Update(double x, double y, double s, double d, double yaw,
                 double speed) {
  this->x = x;
  this->y = y;
  this->s = s;
  this->d = d;
  this->yaw = yaw;
  this->speed = speed;
  UpdateObvervation(s, d);
}

void KinematicsTracker::UpdateObvervation(double pos) {
  _measursments.push_back(pos);

  if (_measursments.size() > 30) {
    _measursments.erase(_measursments.begin());
  }

  if (_measursments.size() > 2) {
    size_t index3 = _measursments.size() - 1;
    size_t index2 = _measursments.size() - 2;
    size_t index1 = _measursments.size() - 3;
    double dist1 = _measursments[index2] - _measursments[index1];
    double dist2 = _measursments[index3] - _measursments[index1];
    double t1 = Configuration::TIME_STEP;
    double t2 = 2 * t1;
    Eigen::Matrix2d A;
    A << t1, 0.5 * t1 * t1, t2, 0.5 * t2 * t2;
    Eigen::Vector2d b;
    b << dist1, dist2;
    Eigen::Vector2d x = A.inverse() * b;
    _values[0] = _measursments[2];
    _values[1] = x[0];
    _values[2] = x[1];
  }
}

void Ego::UpdateObvervation(double s, double d) {
  _sTracker.UpdateObvervation(s);
  auto sValues = _sTracker.GetValues();
  this->s = sValues[0];
  sVel = sValues[1];
  sAcc = sValues[2];

  _dTracker.UpdateObvervation(d);
  auto dValues = _dTracker.GetValues();
  this->d = dValues[0];
  dVel = dValues[1];
  dAcc = dValues[2];
}

// void Vehicle::UpdateFromPerception(const Map::ConstPtr &pMap,
//                                    const Perception &perception) {
//   // TODO
// }

Vehicle Vehicle::CreateFromPerception(const Map::ConstPtr &pMap,
                                      const Perception &perception) {
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

  return Vehicle(perception.id, conf);
}

Vehicle Vehicle::CreateFromEgo(const Map::ConstPtr &pMap, const Ego &ego) {
  return Vehicle(std::numeric_limits<int>::max(),
                 VehicleConfiguration(ego.s, ego.sVel, ego.sAcc, ego.d,
                                      ego.dVel, ego.dAcc));
}

VehicleConfiguration VehicleConfiguration::GetConfiguration(
    const double time) const {
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

VehicleConfiguration Vehicle::GetConfiguration(const double time) const {
  return _conf.GetConfiguration(time);
}

void Vehicle::UpdateObvervation(double s, double d) {
  _sTracker.UpdateObvervation(s);
  auto sValues = _sTracker.GetValues();
  _conf.sPos = sValues[0];
  _conf.sVel = sValues[1];
  _conf.sAcc = sValues[2];

  _dTracker.UpdateObvervation(d);
  auto dValues = _dTracker.GetValues();
  _conf.dPos = dValues[0];
  _conf.dVel = dValues[1];
  _conf.dAcc = dValues[2];
}

}  // namespace pathplanning
