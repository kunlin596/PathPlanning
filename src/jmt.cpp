#include "jmt.h"

#include "Eigen-3.3/Eigen/Dense"
#include "utils.h"

namespace {
using namespace Eigen;
}

namespace pathplanning {

JMTTrajectory1d
JMT::Solve1d_6DoF(const Vector6d& conditions, const double t)
{
  const Vector3d& s_i = conditions.topRows<3>();
  const Vector3d& s_f = conditions.bottomRows<3>();

  double t2 = t * t;
  double t3 = t2 * t;
  double t4 = t3 * t;
  double t5 = t4 * t;

  // clang-format off
  Matrix3d A;
  A <<
    t3       , t4        , t5        ,
    3.0 * t2 , 4.0 * t3  , 5.0 * t4  ,
    6.0 * t  , 12.0 * t2 , 20.0 * t3 ;
  // clang-format on

  Matrix3d M = Matrix3d::Identity();
  M(0, 1) = M(1, 2) = t;
  M(0, 2) = 0.5 * t2;

  Vector3d c012 = { s_i[0], s_i[1], s_i[2] / 2.0 };
  Vector3d c345 = A.colPivHouseholderQr().solve(s_f - M * s_i);

  Vector6d coeffs;
  coeffs << c012[0], c012[1], c012[2], c345[0], c345[1], c345[2];

  JMTTrajectory1d origTraj(coeffs, s_i, s_f, t);
  double clippedTime = std::min(2.0, t);
  Vector3d new_s_f = origTraj(clippedTime).topRows<3>();
  return JMTTrajectory1d(coeffs, s_i, new_s_f, clippedTime);
}

JMTTrajectory1d
JMT::Solve1d_5DoF(const Vector5d& conditions, const double t)
{
  const Vector3d& s_i = conditions.topRows<3>();
  const Vector2d& s_f = conditions.bottomRows<2>();

  double t2 = t * t;
  double t3 = t2 * t;
  double t4 = t3 * t;
  double t5 = t4 * t;
  // clang-format off
  Matrix2d A;
  A <<
    3.0 * t2 , 4.0 * t3   ,
    6.0 * t  , 12.0 * t2  ;
  // clang-format on
  Matrix2d M = Matrix2d::Identity();
  M(0, 1) = t;

  Vector3d c012 = { s_i[0], s_i[1], s_i[2] / 2.0 };
  Vector2d c34 = A.colPivHouseholderQr().solve(s_f - M * s_i.bottomRows<2>());

  Vector6d coeffs;
  coeffs << c012[0], c012[1], c012[2], c34[0], c34[1], 0.0;

  JMTTrajectory1d origTraj(coeffs, s_i, s_f, t);
  double clippedTime = std::min(2.0, t);
  Vector3d new_s_f = origTraj(clippedTime).topRows<3>();
  return JMTTrajectory1d(coeffs, s_i, new_s_f, clippedTime);
}

JMTTrajectory2d
JMT::Solve2d(const Matrix62d& conditions, const double t)
{
  return { JMT::Solve1d_6DoF(conditions.col(0), t), JMT::Solve1d_6DoF(conditions.col(1), t) };
}

bool
JMTTrajectory1d::Validate(const Configuration& conf)
{
  return Validate({ std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN() },
                  conf.speedLimit,
                  conf.trajectory.maxAcc,
                  conf.trajectory.maxJerk,
                  conf.trajectory.timeResolution);
}

bool
JMTTrajectory1d::Validate(const std::array<double, 2>& posBound,
                          double maxVel,
                          double maxAcc,
                          double maxJerk,
                          double totalAccel,
                          double totalJerk,
                          int numPoints)
{
  _avgAccel = 0.0;
  _avgJerk = 0.0;
  _avgVelocity = 0.0;

  double timeResolution = _time / static_cast<double>(numPoints);
  for (double currTime = 0.0; currTime < (_time + 1e-6); currTime += timeResolution) {
    auto values = Eval(currTime);
    if (!std::isnan(posBound[0]) and !std::isnan(posBound[1])) {
      if ((values[0] < posBound[0]) or (posBound[1] < values[0])) {
        _message = fmt::format(
          "position is out of range, {:7.3f} out of [{:7.3f}, {:7.3f}]", values[0], posBound[0], posBound[1]);
        _isvalid = false;
        SPDLOG_DEBUG(_message);
        return _isvalid;
      }
    }

    if (std::abs(values[1]) > maxVel) {
      _message = fmt::format("speed is out of range, {:7.3f} bigger than {:7.3f}", values[1], maxVel);
      _isvalid = false;
      SPDLOG_DEBUG(_message);
      return _isvalid;
    }
    if (std::abs(values[2]) > maxAcc) {
      _message = fmt::format("accel is out of range, {:7.3f} bigger than {:7.3f}", values[2], maxAcc);
      _isvalid = false;
      SPDLOG_DEBUG(_message);
      return _isvalid;
    }
    if (std::abs(values[3]) > maxJerk) {
      _message = fmt::format("jerk is out of range, {:7.3f} bigger than {:7.3f}", values[3], maxJerk);
      _isvalid = false;
      SPDLOG_DEBUG(_message);
      return _isvalid;
    }
    _avgVelocity += std::abs(values[1]) * timeResolution;
    _avgAccel += std::abs(values[2]) * timeResolution;
    _avgJerk += std::abs(values[3]) * timeResolution;
  }

  _avgVelocity /= _time;

  _avgAccel /= _time;
  if (_avgAccel > totalAccel) {
    _isvalid = false;
    _message = fmt::format("total acceleration is out of range, {:7.3f} bigger than {:7.3f}", _avgAccel, totalAccel);
    SPDLOG_DEBUG(_message);
    return _isvalid;
  }

  _avgJerk /= _time;
  if (_avgJerk > totalJerk) {
    _message = fmt::format("total jerk is out of range, {:7.3f} bigger than {:7.3f}", _avgJerk, totalJerk);
    _isvalid = false;
    SPDLOG_DEBUG(_message);
    return _isvalid;
  }

  _isvalid = true;
  return _isvalid;
}

double
JMTTrajectory1d::ComputeCost(double kTime, double kPos, double kJerk, double kEfficiency)
{
  double timeCost = Logistic(_time) * kTime;

  double posCost = Logistic(GetPosition(_time) - GetPosition(0.0)) * kPos;

  double currTime = 0.0;
  double totalJerk = 0.0;
  while (currTime < _time) {
    totalJerk += std::pow(GetJerk(currTime), 2);
    currTime += 0.02;
  }
  double jerkCost = totalJerk * kJerk;

  double efficiencyCost = Logistic(1.0 - _endCond[1] / Mps2Mph(49.0)) * kEfficiency;

  _cost = timeCost + posCost + jerkCost + efficiencyCost;
  return _cost;
}

nlohmann::json
JMTTrajectory1d::Dump() const
{
  return { { "position", GetPositionFn().Dump() },
           { "velocity", GetVelocityFn().Dump() },
           { "acceleration", GetAcclerationFn().Dump() },
           { "jerk", GetJerkFn().Dump() },
           { "snap", GetSnapFn().Dump() },
           { "crackle", GetCrackleFn().Dump() },
           { "time", GetTime() },
           { "cost", GetCost() } };
}

void
JMTTrajectory1d::Write(const std::string& filename) const
{
  utils::WriteJson(filename, Dump());
}

JMTTrajectory2d::JMTTrajectory2d(const JMTTrajectory1d& lonTraj, const JMTTrajectory1d& latTraj)
  : _lonTraj(lonTraj)
  , _latTraj(latTraj)
{
  // assert(lonTraj.GetTime() == latTraj.GetTime());
}

nlohmann::json
JMTTrajectory2d::Dump() const
{
  using namespace nlohmann;
  return { { "lonTraj", _lonTraj.Dump() }, { "latTraj", _latTraj.Dump() } };
}

void
JMTTrajectory2d::Write(const std::string& filename) const
{
  utils::WriteJson(filename, Dump());
}

bool
JMTTrajectory2d::Validate(const Map& map, const Configuration& conf)
{
  if (!_lonTraj.GetIsValid() or !_latTraj.GetIsValid()) {
    _isvalid = false;
    return _isvalid;
  }

  _isvalid = true;
  return _isvalid;
}

Matrix62d
JMTTrajectory2d::Eval(const double t) const
{
  Matrix62d kinematics;
  kinematics.col(0) = _lonTraj(t);
  kinematics.col(1) = _latTraj(t);
  return kinematics;
}

Matrix62d
JMTTrajectory2d::operator()(const double t) const
{
  return Eval(t);
}

std::ostream&
operator<<(std::ostream& out, const pathplanning::JMTTrajectory2d& traj)
{
  return out << fmt::format("JMTTrajectory2d(time={}, sCoeffs={}, dCoeffs={})",
                            traj.GetTime(),
                            traj.GetSFunc().coeffs.transpose(),
                            traj.GetDFunc().coeffs.transpose());
}
} // namespace pathplanning
