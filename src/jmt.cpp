#include "jmt.h"

#include "Eigen-3.3/Eigen/Dense"
#include "utils.h"

namespace {
using namespace Eigen;
}

namespace pathplanning {

JMTTrajectory1d
JMT::Solve1d(const Vector6d& conditions, const double t)
{
  const Vector3d& start = conditions.topRows<3>();
  const Vector3d& end = conditions.bottomRows<3>();

  Vector6d coeffs;

  Matrix3d A;
  double t2 = t * t;
  double t3 = t2 * t;
  double t4 = t3 * t;
  double t5 = t4 * t;
  // clang-format off
  A <<
    t3       , t4        , t5        ,
    3.0 * t2 , 4.0 * t3  , 5.0 * t4  ,
    6.0 * t  , 12.0 * t2 , 20.0 * t3 ;

  Vector3d b = {
      end[0] - (start[0] + start[1] * t + start[2] * t2 / 2.0),
      end[1] - (start[1] + start[2] * t),
      end[2] - start[2]
  };
  // clang-format on

  // See
  // https://eigen.tuxfamily.org/dox-devel/group__TutorialLinearAlgebra.html
  Vector3d x = A.colPivHouseholderQr().solve(b);
  coeffs << start[0], start[1], start[2] / 2.0, x[0], x[1], x[2];
  return JMTTrajectory1d(coeffs, start, end, t);
}

JMTTrajectory2d
JMT::Solve2d(const Matrix62d& conditions, const double t)
{
  return { JMT::Solve1d(conditions.col(0), t), JMT::Solve1d(conditions.col(1), t) };
}

bool
JMTTrajectory1d::IsValid(const Configuration& conf)
{
  return IsValid(conf.speedLimit, conf.trajectory.maxAcc, conf.trajectory.maxJerk, conf.trajectory.timeResolution);
}

bool
JMTTrajectory1d::IsValid(double maxVel, double maxAcc, double maxJerk, double timeResolution)
{
  double currTime = 0.0;
  while (currTime < _time + 1e-6) {
    auto values = Eval(currTime);
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
    currTime += timeResolution;
  }
  _isvalid = true;
  return _isvalid;
}


double
JMTTrajectory1d::ComputeCost(double kTime, double kPos, double kVel, double kAccel, double kJerk)
{
  double timeCost = _time * kTime;
  double posCost = std::pow(GetPosition(_time) - GetPosition(0.0), 2) * kPos;
  double velCost = GaussianLoss1D(20.0, 5.0, GetVelocity(_time)) * kVel;
  double jerkCost = GetJerk(_time) * kJerk;
  return timeCost + posCost + velCost + jerkCost;
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
           { "time", GetTime() } };
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
