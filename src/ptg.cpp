#include "ptg.h"

#include "collision_checker.h"
#include "goalsampler.h"
#include "log.h"
#include "utils.h"

#include <boost/assert.hpp>

#include <pybind11/pybind11.h>

#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#ifdef DEBUG_MODE
#include <boost/filesystem.hpp>
#endif

#ifdef _OPENMP
#include <omp.h>
#endif

namespace {
using namespace pathplanning;
namespace py = pybind11;

/**
 * @brief      Group vehicles into lanes
 *
 * @param[in]  trackedVehicleMap  The tracked vehicle map
 *
 * @return     vehicles per lane map
 */
std::unordered_map<int, std::vector<Vehicle>>
_GroupVehicles(const TrackedVehicleMap& trackedVehicleMap)
{
  std::unordered_map<int, std::vector<Vehicle>> groupedVehicles;
  for (const auto& [id, vehicle] : trackedVehicleMap) {
    int laneId = Map::GetLaneId(vehicle.GetKinematics(0.0)(0, 1));
    if (laneId < 0) {
      continue;
    }
    if (groupedVehicles.count(laneId) == 0) {
      groupedVehicles[id] = std::vector<Vehicle>();
    }
    groupedVehicles[laneId].push_back(vehicle);
  }
  return groupedVehicles;
}

/**
 * @brief      Gets the front back vehicles per lane.
 *
 * @param[in]  egoKinematics       The ego kinematics
 * @param[in]  vehicles            The vehicles
 * @param[in]  nonEgoSearchRadius  The non ego search radius
 *
 * @return     The front back vehicles per lane.
 */
std::unordered_map<std::string, Vehicle>
_GetFrontBackVehiclesPerLane(const Matrix32d& egoKinematics,
                             const std::vector<Vehicle>& vehicles,
                             double nonEgoSearchRadius)
{
  double frontSDiff = std::numeric_limits<double>::max();
  double backSDiff = std::numeric_limits<double>::max();

  int frontId = -1;
  int backId = -1;

  double egoS = egoKinematics(0, 0);

  for (size_t i = 0; i < vehicles.size(); ++i) {
    const Vehicle& vehicle = vehicles[i];
    double s = vehicle.GetKinematics(0.0)(0, 0);
    double currFrontSDiff = s - egoS;
    double currBackSDiff = egoS - s;

    if (0.0 < currFrontSDiff and currFrontSDiff < frontSDiff) {
      frontSDiff = currFrontSDiff;
      frontId = i;

    } else if (0.0 < currBackSDiff and currBackSDiff < backSDiff) {
      backSDiff = currBackSDiff;
      backId = i;
    }
  }

  std::unordered_map<std::string, Vehicle> result;

  if (frontId != -1 and frontSDiff < nonEgoSearchRadius) {
    result["front"] = vehicles[frontId];
    SPDLOG_TRACE("Found front id {:2d}, frontSDiff={:7.3f}", vehicles[frontId].GetId(), frontSDiff);
  }

  if (backId != -1 and backSDiff < nonEgoSearchRadius) {
    result["back"] = vehicles[backId];
    SPDLOG_TRACE("Found back id {:2d}, backSDiff={:7.3f}", vehicles[backId].GetId(), backSDiff);
  }

  return result;
}

void
_GetOptimalCombination(const std::vector<JMTTrajectory1d>& lonTrajs,
                       const std::vector<JMTTrajectory1d>& latTrajs,
                       int& lonId,
                       int& latId,
                       double& minCost)
{
  if ((lonTrajs.size() == 0) || (latTrajs.size() == 0)) {
    lonId = -1;
    latId = -1;
    return;
  }

  // cost weight for longitudinal / lateral
  double kLon = 1.0;
  double kLat = 2.0;

  // build sum matrix
  Eigen::MatrixXd sdCostSum(lonTrajs.size(), latTrajs.size());
  for (int row = 0; row < lonTrajs.size(); ++row) {
    for (int col = 0; col < latTrajs.size(); ++col) {
      sdCostSum(row, col) = kLon * lonTrajs[row].GetCost() + kLat * latTrajs[col].GetCost();
    }
  }
  // find minimum
  minCost = sdCostSum.minCoeff(&lonId, &latId);
}

}

namespace pathplanning {

PolynomialTrajectoryGenerator::PolynomialTrajectoryGenerator(const Map& map, const Configuration& conf)
  : _map(map)
  , _conf(conf)
{}

void
PolynomialTrajectoryGenerator::_GenerateLonTrajectory(const LongitudinalManeuverType& lonBehavior,
                                                      const Ego& ego,
                                                      const Vehicle& vehicle,
                                                      std::vector<JMTTrajectory1d>& trajectories)
{
  switch (lonBehavior) {
    case LongitudinalManeuverType::kVelocityKeeping:
      _GenerateVelocityKeepingTrajectory(ego, trajectories);
      return;
    case LongitudinalManeuverType::kStopping:
      _GenerateStoppingTrajectory(ego, trajectories);
      return;
    case LongitudinalManeuverType::kFollowing:
      if (vehicle.GetId() != -1) {
        _GenerateVehicleFollowingTrajectory(ego, vehicle, trajectories);
      } else {
        _GenerateVelocityKeepingTrajectory(ego, trajectories);
      }
      return;
  }
}

void
PolynomialTrajectoryGenerator::_GenerateLatTrajectory(const LateralManeuverType& latBehavior,
                                                      const Ego& ego,
                                                      std::vector<JMTTrajectory1d>& trajectories)
{

  Vector3d egoLatKinematics = ego.GetKinematics(0.0).block<3, 1>(0, 1);
  double targetD = Map::GetLaneCenterD(Map::GetLaneId(egoLatKinematics[0]));

  switch (latBehavior) {
    case LateralManeuverType::kLeftLaneChanging:
      targetD = Map::GetLaneCenterD(Map::GetLaneId(targetD) - 1);
      break;
    case LateralManeuverType::kRightLaneChanging:
      targetD = Map::GetLaneCenterD(Map::GetLaneId(targetD) + 1);
      break;
  }

  std::vector<double> targetTimeList = { 3.0, 3.5, 4.0 };
  _SolveFullConstraints1d(
    // Constraints
    egoLatKinematics[0], // d_i
    egoLatKinematics[1], // d_i dot
    egoLatKinematics[2], // d_i dot dot
    targetD,             // d_f
    0.0,                 // d_f dot
    0.0,                 // d_f dot dot
    // Time
    targetTimeList,
    // S deltas
    { 0.0 }, // delta
    // Weights
    1.0,    // time
    1.0,    // pos
    10.0,   // vel
    100.0,  // acc
    1000.0, // jerk
    // Outputs
    trajectories);
}

void
PolynomialTrajectoryGenerator::_SolveFullConstraints1d(double s0,
                                                       double s0dot,
                                                       double s0ddot,
                                                       double s1,
                                                       double s1dot,
                                                       double s1ddot,
                                                       const std::vector<double>& targetTimeList,
                                                       const std::vector<double>& dsList,
                                                       double kTime,
                                                       double kPos,
                                                       double kVel,
                                                       double kAcc,
                                                       double kJerk,
                                                       std::vector<JMTTrajectory1d>& trajectories)
{
  Vector6d conditions;
  conditions << s0, s0dot, s0ddot, s1, s1dot, s1ddot;
  for (double ds : dsList) {
    conditions[3] = s1 + ds;
    // SPDLOG_DEBUG("Target {}", conditions[3]);
    for (double Tj : targetTimeList) {
      auto traj = JMT::Solve1d(conditions, Tj);
      if (!traj.IsValid(_conf))
        continue;
      traj.ComputeCost(kTime, kPos, kVel, kAcc, kJerk);
      trajectories.push_back(traj);
    }
  }
}

void
PolynomialTrajectoryGenerator::_GenerateVelocityKeepingTrajectory(const Ego& ego,
                                                                  std::vector<JMTTrajectory1d>& trajectories)
{
  std::vector<double> ds1List;
  std::vector<double> targetTimeList;

  Vector3d egoLonKinematics = ego.GetKinematics(0.0).block<3, 1>(0, 0);
  double lonVelocity = egoLonKinematics[1];

  double targetVelocity = 0.0;
  if (lonVelocity < 5.0) {
    targetVelocity = 5.0;
    ds1List = { 1.0, 2.5, 5.0, 7.5, 10.0, 15.0 };
    targetTimeList = { 3.0, 4.0 };
  } else if (lonVelocity < 10.0) {
    targetVelocity = 10.0;
    ds1List = { 7.5, 10.0, 15.0, 20.0, 25.0 };
    targetTimeList = { 3.0, 4.0 };
  } else {
    targetVelocity = 20.0;
    ds1List = { 15.0, 20.0, 25.0, 30.0 };
    targetTimeList = { 2.0, 3.0, 4.0, 5.0 };
  }

  _SolveFullConstraints1d(
    // Constraints
    egoLonKinematics[0],
    egoLonKinematics[1],
    egoLonKinematics[2],
    egoLonKinematics[0],
    targetVelocity,
    0.0,
    // Time
    targetTimeList,
    // S deltas
    ds1List,
    // Weights
    1.0,
    10.0,
    1.0,
    1.0,
    10.0,
    // Outputs
    trajectories);
}

void
PolynomialTrajectoryGenerator::_GenerateVehicleFollowingTrajectory(const Ego& ego,
                                                                   const Vehicle& vehicle,
                                                                   std::vector<JMTTrajectory1d>& trajectories)
{
  // TODO
}

void
PolynomialTrajectoryGenerator::_GenerateStoppingTrajectory(const Ego& ego, std::vector<JMTTrajectory1d>& trajectories)
{
  // TODO
}

Matrix32d
PolynomialTrajectoryGenerator::ComputeStartState(const Vehicle& ego,
                                                 const JMTTrajectory2d& prevTraj,
                                                 const Waypoints& prevPath,
                                                 int numPointsToPreserve,
                                                 bool usePython)
{
  if (usePython) {
    return _ComputeStartStatePy(ego, prevTraj, prevPath, numPointsToPreserve);
  }

  if (prevPath.empty()) {
    return ego.GetKinematics(0.0);
  }

  double executedTime = 0.0;
  if (prevPath.size() > 0) {
    executedTime = (_conf.numPoints - prevPath.size() + size_t(numPointsToPreserve)) * _conf.simulator.timeStep;
  }

  SPDLOG_DEBUG("prevPath.size()={:d}, executedTime={:.3f}", prevPath.size(), executedTime);
  return prevTraj(executedTime).topRows<3>();
}

JMTTrajectory2d
PolynomialTrajectoryGenerator::GenerataTrajectory(const Ego& ego,
                                                  const TrackedVehicleMap& trackedVehicleMap,
                                                  bool usePython)
{
  // For more information on trajectory generation, see
  // "Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame",
  // M. Werling, J. Ziegler, S. Kammel and S. Thrun, ICRA 2010

  if (usePython) {
    return _GenerataTrajectoryPy(ego, trackedVehicleMap);
  }

  auto groupedVehicles = _GroupVehicles(trackedVehicleMap);
  auto egoLaneId = Map::GetLaneId(ego.GetKinematics(0.0)(0, 1));

  // Figure out the lanes for planning
  int leftLaneId = egoLaneId - 1;
  int rightLaneId = egoLaneId + 1;
  std::unordered_map<std::string, int> laneIdsForPlanning;
  laneIdsForPlanning["current"] = egoLaneId;

#ifdef LANE_CHANGING
  if (0 <= leftLaneId and leftLaneId < egoLaneId) {
    laneIdsForPlanning["left"] = leftLaneId;
  }
  if (egoLaneId < rightLaneId and rightLaneId < Map::NUM_LANES) {
    laneIdsForPlanning["right"] = rightLaneId;
  }
#endif

  // Figure out lat behaviors
  std::unordered_map<int, LateralManeuverType> latBehaviors;
  latBehaviors[egoLaneId] = LateralManeuverType::kLaneKeeping;
  if (laneIdsForPlanning.count("left")) {
    latBehaviors[leftLaneId] = LateralManeuverType::kLeftLaneChanging;
  }
  if (laneIdsForPlanning.count("right")) {
    latBehaviors[rightLaneId] = LateralManeuverType::kRightLaneChanging;
  }

  // Figure out lon behaviors
  std::unordered_map<int, std::vector<LongitudinalManeuverType>> lonBehaviors;
  std::unordered_map<int, Vehicle> leadingVehicles;

  for (const auto& [laneName, laneId] : laneIdsForPlanning) {
    lonBehaviors[laneId] = {
      LongitudinalManeuverType::kVelocityKeeping,
      // LongitudinalManeuverType::kStopping
    };
    if (groupedVehicles.count(laneId)) {
      const auto& vehicles = groupedVehicles[laneId];
      SPDLOG_TRACE("Searching for nearby vehicle on lane {:d}", laneId);
      auto frontBackVehicles = _GetFrontBackVehiclesPerLane(ego.GetKinematics(0.0), vehicles, 60.0);
      if (frontBackVehicles.count("front")) {
        leadingVehicles[laneId] = frontBackVehicles["front"];
        lonBehaviors[laneId].push_back(LongitudinalManeuverType::kFollowing);
      } else {
        leadingVehicles[laneId] = Vehicle();
      }
    }
  }

  JMTTrajectory2d bestTraj;
  double minCost = std::numeric_limits<double>::min();
  static int step = 0;

  for (const auto& [laneName, laneId] : laneIdsForPlanning) {

    // Generate trajectories for ego lane and adjacent lanes
    std::vector<JMTTrajectory1d> lonTrajs;
    std::vector<JMTTrajectory1d> latTrajs;

    // Generate lon trajectories
    for (const auto& behavior : lonBehaviors[laneId]) {
      _GenerateLonTrajectory(behavior, ego, leadingVehicles[laneId], lonTrajs);
    }

    // Generate lat trajectories
    _GenerateLatTrajectory(latBehaviors[laneId], ego, latTrajs);

    if (lonTrajs.empty() or latTrajs.empty()) {
      SPDLOG_DEBUG("Planning failed for lane {:d}", laneId);
      continue;
    }

#ifdef DEBUG_MODE
    SPDLOG_DEBUG("DEBUG_MODE OUTPUT.");
    namespace bfs = boost::filesystem;
    bfs::path trajPath("/tmp/traj");
    if (bfs::exists(trajPath) and bfs::is_directory(trajPath)) {
      bfs::remove(trajPath);
    }
    bfs::create_directory(trajPath);

    for (size_t i = 0; i < lonTrajs.size(); ++i) {
      std::string lonfilename = fmt::format("{:d}_{:d}_{:d}_lontraj.json", step, laneId, i);
      utils::WriteJson(trajPath / lonfilename, lonTrajs[i].Dump());
    }
#endif

    int lonId = -1, latId = -1;
    double cost;
    _GetOptimalCombination(lonTrajs, latTrajs, lonId, latId, cost);

    const auto& bestLonTraj = lonTrajs[lonId];
    const auto& bestLatTraj = latTrajs[latId];
    auto traj = JMTTrajectory2d(bestLonTraj, bestLatTraj);

    auto [collidedVehicleId, distance] = CollisionChecker::IsInCollision(traj, trackedVehicleMap, _conf);
    if (collidedVehicleId != -1) {
      continue;
    }

    if (cost < minCost) {
      minCost = cost;
      bestTraj = traj;
    }
  }

  step++;

  return bestTraj;
}

Matrix32d
PolynomialTrajectoryGenerator::_ComputeStartStatePy(const Vehicle& ego,
                                                    const JMTTrajectory2d& prevTraj,
                                                    const Waypoints& prevPath,
                                                    int numPointsToPreserve)
{
  py::module pyPTG = py::module::import("build.ptg");
  py::module pyJson = py::module::import("json");
  py::array_t<double> pyStartState = pyPTG.attr("compute_start_state")(pyJson.attr("loads")(ego.Dump().dump()),
                                                                       pyJson.attr("loads")(prevTraj.Dump().dump()),
                                                                       prevPath,
                                                                       numPointsToPreserve);
  // py::print(pyStartState);
  auto r = pyStartState.unchecked<2>();
  Matrix32d startState;
  for (py::ssize_t i = 0; i < r.shape(0); i++)
    for (py::ssize_t j = 0; j < r.shape(1); j++)
      startState(i, j) = r(i, j);
  // SPDLOG_DEBUG(startState);
  return startState;
}

JMTTrajectory2d
PolynomialTrajectoryGenerator::_GenerataTrajectoryPy(const Ego& ego, const TrackedVehicleMap& trackedVehicleMap)
{
  py::module pyPTG = py::module::import("build.ptg");
  py::dict pyTrackedVehicleMap;
  py::module pyJson = py::module::import("json");
  for (const auto& [id, vehicle] : trackedVehicleMap) {
    pyTrackedVehicleMap[std::to_string(id).c_str()] = pyJson.attr("loads")(vehicle.Dump().dump());
  }
  py::dict pyTraj2d = pyPTG.attr("generate_trajectory")(pyJson.attr("loads")(ego.Dump().dump()), pyTrackedVehicleMap);

  // TODO Fix py array to eigen conversion
  Vector6d lonCoef =
    Eigen::Map<const Vector6d>(reinterpret_cast<const double*>(pyTraj2d["lon_coef"].cast<py::array>().data()), 6, 1);
  Vector3d lonStartCond = Eigen::Map<const Vector3d>(
    reinterpret_cast<const double*>(pyTraj2d["lon_start_cond"].cast<py::array>().data()), 3, 1);
  Vector3d lonEndCond = Eigen::Map<const Vector3d>(
    reinterpret_cast<const double*>(pyTraj2d["lon_end_cond"].cast<py::array>().data()), 3, 1);
  double lonTime = pyTraj2d["lon_time"].cast<double>();

  Vector6d latCoef =
    Eigen::Map<const Vector6d>(reinterpret_cast<const double*>(pyTraj2d["lat_coef"].cast<py::array>().data()), 6, 1);
  Vector3d latStartCond = Eigen::Map<const Vector3d>(
    reinterpret_cast<const double*>(pyTraj2d["lat_start_cond"].cast<py::array>().data()), 3, 1);
  Vector3d latEndCond = Eigen::Map<const Vector3d>(
    reinterpret_cast<const double*>(pyTraj2d["lat_end_cond"].cast<py::array>().data()), 3, 1);
  double latTime = pyTraj2d["lat_time"].cast<double>();

  return JMTTrajectory2d(JMTTrajectory1d(QuinticFunctor(lonCoef), lonStartCond, lonEndCond, lonTime),
                         JMTTrajectory1d(QuinticFunctor(latCoef), latStartCond, latEndCond, latTime));
}

} // namespace pathplanning
