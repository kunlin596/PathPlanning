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
JMTTrajectory1d::IsValid(const Configuration& conf) const
{
  double currTime = 0.0;
  while (currTime < _time + 1e-6) {
    auto values = Eval(currTime);
    if (std::abs(values[1]) > conf.speedLimit) {
      SPDLOG_TRACE("speed is out of range, {:7.3f} bigger than {:7.3f}", values[1], conf.speedLimit);
      return false;
    }
    if (std::abs(values[2]) > conf.trajectory.maxAcc) {
      SPDLOG_TRACE("accel is out of range, {:7.3f} bigger than {:7.3f}", values[2], conf.trajectory.maxAcc);
      return false;
    }
    if (std::abs(values[3]) > conf.trajectory.maxJerk) {
      SPDLOG_TRACE("jerk is out of range, {:7.3f} bigger than {:7.3f}", values[3], conf.trajectory.maxJerk);
      return false;
    }
    currTime += conf.trajectory.timeResolution;
  }
  return true;
}

nlohmann::json
JMTTrajectory1d::Dump() const
{
  return { { "func5", GetFunc5().Dump() }, { "func4", GetFunc4().Dump() }, { "func3", GetFunc3().Dump() },
           { "func2", GetFunc2().Dump() }, { "func1", GetFunc1().Dump() }, { "func0", GetFunc0().Dump() },
           { "time", GetTime() } };
}

void
JMTTrajectory1d::Write(const std::string& filename) const
{
  utils::WriteJson(filename, Dump());
}

double
JMTTrajectory2d::GetNearestApproachTo(const Vehicle& vehicle, double maxTimeDuration, double timeStep) const
{
  double minDist = std::numeric_limits<double>::infinity();
  double currTime = 0.0;
  while (currTime < maxTimeDuration) {
    currTime += timeStep;
    Matrix62d trajKinematics = Eval(currTime);
    Matrix32d vehicleKinematics = vehicle.GetKinematics(currTime);

    double dist =
      GetDistance({ trajKinematics(0, 0), trajKinematics(0, 1) }, { vehicleKinematics(0, 0), vehicleKinematics(0, 1) });

    if (dist < minDist) {
      minDist = dist;
    }
  }
  return minDist;
}

double
JMTTrajectory2d::GetNearestApproachTo(const std::vector<Vehicle>& vehicles,
                                      double maxTimeDuration,
                                      double timeStep) const
{
  double minDist = std::numeric_limits<double>::infinity();
  for (const auto& v : vehicles) {
    double dist = GetNearestApproachTo(v, maxTimeDuration, timeStep);
    if (dist < minDist) {
      minDist = dist;
    }
  }
  return minDist;
}

double
JMTTrajectory2d::GetNearestApproachTo(const std::unordered_map<int, Vehicle>& vehicles,
                                      double maxTimeDuration,
                                      double timeStep) const
{
  double minDist = std::numeric_limits<double>::infinity();
  for (const auto& [id, v] : vehicles) {
    double dist = GetNearestApproachTo(v, maxTimeDuration, timeStep);
    if (dist < minDist) {
      minDist = dist;
    }
  }
  return minDist;
}

JMTTrajectory2d::JMTTrajectory2d(const JMTTrajectory1d& traj1, const JMTTrajectory1d& traj2)
  : _traj1(traj1)
  , _traj2(traj2)
{
  assert(traj1.GetTime() == traj2.GetTime());
}

nlohmann::json
JMTTrajectory2d::Dump() const
{
  using namespace nlohmann;
  return { { "traj1", _traj1.Dump() }, { "traj2", _traj2.Dump() } };
}

void
JMTTrajectory2d::Write(const std::string& filename) const
{
  utils::WriteJson(filename, Dump());
}

bool
JMTTrajectory2d::IsValid(const Map& map, const Configuration& conf) const
{
  if (not _traj1.IsValid(conf) or not _traj2.IsValid(conf)) {
    return false;
  }

  double currTime = 0.0;
  std::vector<Waypoint> sampledWaypoints;
  while (currTime < GetTime() + 1e-6) {
    auto kinematics = Eval(currTime);
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
      SPDLOG_TRACE("curvature is {:7.3f} exceeding threashold {:7.3f}", curvature, conf.trajectory.maxCurvature);
      return false;
    }
  }
  return true;
}

Matrix62d
JMTTrajectory2d::Eval(const double t) const
{
  Matrix62d kinematics;
  kinematics.col(0) = _traj1(t);
  kinematics.col(1) = _traj2(t);
  return kinematics;
}

std::vector<JMTTrajectory2d>
JMT::SolveMultipleFeasible2d(const Matrix62d& conditions, const Map& map, const Configuration& conf)
{
  double minT = (conditions(3, 0) - conditions(0, 0)) / conf.speedLimit;
  std::vector<JMTTrajectory2d> trajs;
  double currTime = minT;
  while (currTime < (minT + conf.trajectory.maxTime + 1e-6)) {
    auto traj = JMT::Solve2d(conditions, currTime);
    if (traj.IsValid(map, conf)) {
      trajs.push_back(traj);
      SPDLOG_TRACE("currTime={}, traj={}", currTime, traj);
    }
    currTime += conf.timeStep;
  }
  return trajs;
}

std::ostream&
operator<<(std::ostream& out, const pathplanning::JMTTrajectory2d& traj)
{
  return out << fmt::format(
           "JMTTrajectory2d(time={}, sCoeffs={}, dCoeffs{})", traj.GetTime(), traj.GetSFunc(), traj.GetDFunc());
}

} // namespace pathplanning
