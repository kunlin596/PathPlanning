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
  const Vector3d& start = conditions.topRows<3>();
  const Vector3d& end = conditions.bottomRows<3>();

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
  M(1, 1) = 0.5 * t2;

  Vector3d c012 = { start[0], start[1], start[2] / 2.0 };
  Vector3d c345 = A.colPivHouseholderQr().solve(end - M * c012);

  Vector6d coeffs;
  coeffs << c012[0], c012[1], c012[2], c345[0], c345[1], c345[2];

  return JMTTrajectory1d(coeffs, start, end, t);
}

JMTTrajectory1d
JMT::Solve1d_5DoF(const Vector5d& conditions, const double t)
{
  const Vector3d& start = conditions.topRows<3>();
  const Vector2d& end = conditions.bottomRows<2>();

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
  M(1, 1) = t;

  Vector3d c012 = { start[0], start[1], start[2] / 2.0 };
  Vector2d c34 = A.colPivHouseholderQr().solve(end - M * c012.bottomRows<2>());

  Vector6d coeffs;
  coeffs << c012[0], c012[1], c012[2], c34[0], c34[1], 0.0;

  return JMTTrajectory1d(coeffs, start, end, t);
}

JMTTrajectory2d
JMT::Solve2d(const Matrix62d& conditions, const double t)
{
  return { JMT::Solve1d_6DoF(conditions.col(0), t), JMT::Solve1d_6DoF(conditions.col(1), t) };
}

bool
JMTTrajectory1d::IsValid(const Configuration& conf)
{
  return IsValid(conf.speedLimit, conf.trajectory.maxAcc, conf.trajectory.maxJerk, conf.trajectory.timeResolution);
}

bool
JMTTrajectory1d::IsValid(double maxVel,
                         double maxAcc,
                         double maxJerk,
                         double totalAccel,
                         double totalJerk,
                         double timeResolution)
{
  double currTime = 0.0;
  double prevPos = std::numeric_limits<double>::min();
  double averAccel = 0.0;
  double averJerk = 0.0;
  while (currTime < _time + 1e-6) {
    auto values = Eval(currTime);
    if (values[0] > prevPos) {
      prevPos = values[0];
    } else {
      _isvalid = false;
      return _isvalid;
    }

    if (std::abs(values[1]) > maxVel) {
      SPDLOG_TRACE("speed is out of range, {:7.3f} bigger than {:7.3f}", values[1], maxVel);
      _isvalid = false;
      return _isvalid;
    }
    if (std::abs(values[2]) > maxAcc) {
      SPDLOG_TRACE("accel is out of range, {:7.3f} bigger than {:7.3f}", values[2], maxAcc);
      _isvalid = false;
      return _isvalid;
    }
    if (std::abs(values[3]) > maxJerk) {
      SPDLOG_TRACE("jerk is out of range, {:7.3f} bigger than {:7.3f}", values[3], maxJerk);
      _isvalid = false;
      return _isvalid;
    }
    averAccel += std::abs(values[2]) * timeResolution;
    averJerk += std::abs(values[3]) * timeResolution;
    currTime += timeResolution;
  }

  averAccel /= this->_time;
  if (averAccel > totalAccel) {
    _isvalid = false;
    return _isvalid;
  }

  averJerk /= this->_time;
  if (averJerk > totalJerk) {
    _isvalid = false;
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
JMTTrajectory2d::IsValid(const Map& map, const Configuration& conf)
{
  if (not _lonTraj.IsValid(conf) or not _latTraj.IsValid(conf)) {
    _isvalid = false;
    return _isvalid;
  }

  double currTime = 0.0;
  std::vector<Waypoint> sampledWaypoints;

  auto roadBoundaries = Map::GetRoadBoundary();
  roadBoundaries[0] -= Vehicle::Size;
  roadBoundaries[1] += Vehicle::Size;

  while (currTime < GetTime() + 1e-6) {
    auto kinematics = Eval(currTime);

    if (roadBoundaries[0] > kinematics(0, 1) or kinematics(0, 1) > roadBoundaries[1]) {
      SPDLOG_WARN("trajectory is off road, d={:7.3f}, roadBoundaries={}", kinematics(0, 1), roadBoundaries);
      _isvalid = false;
      return _isvalid;
    }

    sampledWaypoints.push_back(map.GetXY(kinematics(0, 0), kinematics(0, 1)));
    currTime += conf.trajectory.timeResolution;
  }

  for (size_t i = 1; i < sampledWaypoints.size() - 1; ++i) {
    auto point1 = sampledWaypoints[i - 1];
    auto point2 = sampledWaypoints[i];
    auto point3 = sampledWaypoints[i + 1];

    // Compute the curvature between point 2 and 3
    double heading1 = GetAngle(point1, point2);
    double heading2 = GetAngle(point2, point3);
    double dist = GetDistance(point3, point2);
    double curvature = Rad2Deg(heading2 - heading1) / dist;
    if (curvature > conf.trajectory.maxCurvature) {
      SPDLOG_TRACE("curvature is {:7.3f}, threashold {:7.3f}", curvature, conf.trajectory.maxCurvature);
      _isvalid = false;
      return _isvalid;
    }
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

std::vector<JMTTrajectory2d>
JMT::SolveMultipleFeasible2d(const Matrix62d& conditions, const Map& map, const Configuration& conf)
{
  std::vector<JMTTrajectory2d> trajs;
  int cnt = 0;

  double minT = (conditions(3, 0) - conditions(0, 0)) / conf.speedLimit;
  double currTime = minT;
  while (currTime < (minT + conf.trajectory.maxTime + 1e-6)) {
    auto traj = JMT::Solve2d(conditions, currTime);
    traj.Write(fmt::format("/tmp/jmt/{}.json", cnt));
    if (traj.IsValid(map, conf)) {
      trajs.push_back(traj);
    }
    currTime += conf.trajectory.timeResolution;
    ++cnt;
  }
  if (trajs.empty()) {
    SPDLOG_WARN("Checked {} time steps, and found {} feasible trajectories", cnt, trajs.size());
  } else {
    SPDLOG_DEBUG("Checked {} time steps, and found {} feasible trajectories", cnt, trajs.size());
  }
  return trajs;
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
