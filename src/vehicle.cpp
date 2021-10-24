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
  _Collect(s, d);
}

void Ego::Update(double x, double y, double s, double d, double yaw,
                 double speed) {
  this->x = x;
  this->y = y;
  this->s = s;
  this->d = d;
  this->yaw = yaw;
  this->speed = speed;
  _Collect(s, d);
}

void Ego::_Collect(double s, double d) {
  _sMeasursments.push_back(s);
  if (_sMeasursments.size() > 3) {
    _sMeasursments.erase(_sMeasursments.begin());
  }

  if (_sMeasursments.size() == 3) {
    double dist1 = _sMeasursments[1] - _sMeasursments[0];
    double dist2 = _sMeasursments[2] - _sMeasursments[0];
    double t1 = Configuration::TIME_STEP;
    double t2 = 2 * t1;
    Eigen::Matrix2d A;
    A << t1, 0.5 * t1 * t1, t2, 0.5 * t2 * t2;
    Eigen::Vector2d b;
    b << dist1, dist2;
    Eigen::Vector2d x = A.inverse() * b;
    sVel = x[0];
    sAcc = x[1];
    // SPDLOG_INFO("s={}, sVel={}, sAcc={}", _sMeasursments[2], sVel, sAcc);
  }

  _dMeasursments.push_back(d);
  if (_dMeasursments.size() > 3) {
    _dMeasursments.erase(_dMeasursments.begin());
  }

  if (_dMeasursments.size() == 3) {
    double dist1 = _dMeasursments[1] - _dMeasursments[0];
    double dist2 = _dMeasursments[2] - _dMeasursments[0];
    double t1 = Configuration::TIME_STEP;
    double t2 = 2 * t1;
    Eigen::Matrix2d A;
    A << t1, 0.5 * t1 * t1, t2, 0.5 * t2 * t2;
    Eigen::Vector2d b;
    b << dist1, dist2;
    Eigen::Vector2d x = A.inverse() * b;
    dVel = x[0];
    dAcc = x[1];
    // SPDLOG_INFO("d={}, dVel={}, dAcc={}", _dMeasursments[2], dVel, dAcc);
  }
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

VehicleConfiguration Vehicle::GetConfiguration(const double time) const {
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

}  // namespace pathplanning
