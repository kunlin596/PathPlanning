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
std::pair<std::unordered_map<std::string, Vehicle>, double>
_FindLeadingVehicle(const Matrix32d& egoKinematics, const std::vector<Vehicle>& vehicles, double nonEgoSearchRadius)
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

  return std::make_pair(result, frontSDiff);
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
    case LongitudinalManeuverType::kCrusing:
      _GenerateCrusingTrajectory(ego, trajectories);
      return;
    case LongitudinalManeuverType::kStopping:
      _GenerateStoppingTrajectory(ego, trajectories);
      return;
    case LongitudinalManeuverType::kFollowing:
      _GenerateVehicleFollowingTrajectory(ego, vehicle, trajectories);
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

  Vector6d conditions;
  conditions.block<3, 1>(0, 0) = egoLatKinematics;
  conditions.block<3, 1>(3, 0) << targetD, 0.0, 0.0;

  double timeStep = 10.0 / 20.0;

#ifdef _OPENMP
#pragma omp parallel for
#endif
  for (int i = 0; i < 20; ++i) {
    double T = timeStep * static_cast<double>(i);
    auto traj = JMT::Solve1d_6DoF(conditions, T);
    if (traj.IsValid()) {
      traj.ComputeCost(1.0, 1.0, 2.0, 1.0);
      trajectories.push_back(traj);
    }
  }
}

void
PolynomialTrajectoryGenerator::_GenerateCrusingTrajectory(const Ego& ego, std::vector<JMTTrajectory1d>& trajectories)
{
  Vector3d egoLonKinematics = ego.GetKinematics(0.0).block<3, 1>(0, 0);
  double timeStep = 10.0 / 20.0;
  double startTime = 5.0;

#ifdef _OPENMP
#pragma omp parallel for
#endif
  for (int i = 0; i < 20; ++i) {
    double T = startTime + timeStep * static_cast<double>(i);

    double startSddot = 5.0;
    double sddotStep = 10.0 / 10.0;

    for (int j = 0; j < 20; ++j) {
      double sddot = startSddot + sddotStep * static_cast<double>(j);
      Vector5d conditions;
      conditions.block<3, 1>(0, 0) = egoLonKinematics;
      conditions.block<2, 1>(3, 0) << std::min(egoLonKinematics[1] + sddot, Mph2Mps(50.0)), 0.0;
      auto traj = JMT::Solve1d_5DoF(conditions, T);
      if (traj.IsValid()) {
        traj.ComputeCost(2.0, 2.0, 1.0, 2.0);
        trajectories.push_back(traj);
      }
    }
  }
}

void
PolynomialTrajectoryGenerator::_GenerateVehicleFollowingTrajectory(const Ego& ego,
                                                                   const Vehicle& vehicle,
                                                                   std::vector<JMTTrajectory1d>& trajectories)
{
  Vector3d egoLonKinematics = ego.GetKinematics(0.0).block<3, 1>(0, 0);
  double timeStep = 5.0 / 20.0;
  double startTime = 5.0;

  double tau = 0.1;
  double offset = 30.0;

#ifdef _OPENMP
#pragma omp parallel for
#endif
  for (int i = 0; i < 20; ++i) {
    double T = startTime + timeStep * static_cast<double>(i);
    Vector3d leadingLonKinematics = vehicle.GetKinematics(T).col(0);
    Vector6d conditions;
    conditions.block<3, 1>(0, 0) = egoLonKinematics;
    // clang-format off
    conditions.block<3, 1>(3, 0) <<
      leadingLonKinematics[0] - tau * leadingLonKinematics[1] - offset,
      leadingLonKinematics[1] - tau * leadingLonKinematics[2],
      leadingLonKinematics[2];
    // clang-format on
    auto traj = JMT::Solve1d_6DoF(conditions, T);
    if (traj.IsValid()) {
      traj.ComputeCost(5.0, 2.0, 1.0, 5.0);
      trajectories.push_back(traj);
    }
  }
}

void
PolynomialTrajectoryGenerator::_GenerateStoppingTrajectory(const Ego& ego, std::vector<JMTTrajectory1d>& trajectories)
{
  Vector3d egoLonKinematics = ego.GetKinematics(0.0).block<3, 1>(0, 0);
  double timeStep = 10.0 / 20.0;
  double startTime = 5.0;

#ifdef _OPENMP
#pragma omp parallel for
#endif
  for (int i = 0; i < 20; ++i) {
    double T = startTime + timeStep * static_cast<double>(i);

    double startSddot = 5.0;
    double sddotStep = 10.0 / 10.0;

    for (int j = 0; j < 20; ++j) {
      double sddot = startSddot - sddotStep * static_cast<double>(j);
      Vector5d conditions;
      conditions.block<3, 1>(0, 0) = egoLonKinematics;
      conditions.block<2, 1>(3, 0) << std::max(egoLonKinematics[1] + sddot, 0.0), 0.0;
      auto traj = JMT::Solve1d_5DoF(conditions, T);
      if (traj.IsValid()) {
        traj.ComputeCost(2.0, 2.0, 1.0, 2.0);
        trajectories.push_back(traj);
      }
    }
  }
}

JMTTrajectory2d
PolynomialTrajectoryGenerator::GenerataTrajectory(const Ego& ego, const TrackedVehicleMap& trackedVehicleMap)
{
  // For more information on trajectory generation, see
  // "Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame",
  // M. Werling, J. Ziegler, S. Kammel and S. Thrun, ICRA 2010

  // if (usePython) {
  //   return _GenerataTrajectoryPy(ego, trackedVehicleMap);
  // }

  auto groupedVehicles = _GroupVehicles(trackedVehicleMap);
  auto egoLaneId = Map::GetLaneId(ego.GetKinematics(0.0)(0, 1));

  // Figure out the lanes for planning
  int leftLaneId = egoLaneId - 1;
  int rightLaneId = egoLaneId + 1;

  std::unordered_map<std::string, int> laneIdsForPlanning;
  laneIdsForPlanning["current"] = egoLaneId;

  if (0 <= leftLaneId and leftLaneId < egoLaneId) {
    laneIdsForPlanning["left"] = leftLaneId;
  }
  if (egoLaneId < rightLaneId and rightLaneId < Map::NUM_LANES) {
    laneIdsForPlanning["right"] = rightLaneId;
  }

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
  std::unordered_map<int, Vehicle> leadingVehicles;
  std::unordered_map<int, std::vector<LongitudinalManeuverType>> lonBehaviors;
  for (const auto& [laneName, laneId] : laneIdsForPlanning) {
    lonBehaviors[laneId] = std::vector<LongitudinalManeuverType>();

    if (groupedVehicles.count(laneId)) {
      const auto& vehicles = groupedVehicles[laneId];
      if (vehicles.empty())
        continue;

      const auto& [leadingVehicle, minDist] =
        _FindLeadingVehicle(ego.GetKinematics(0.0), vehicles, _conf.tracker.nonEgoSearchRadius);

      if (leadingVehicle.count("front")) {
        if (minDist < 10.0) {
          lonBehaviors[laneId].push_back(LongitudinalManeuverType::kStopping);
        } else if (10.0 < minDist and minDist < 50.0) {
          leadingVehicles[laneId] = leadingVehicle.at("front");
          lonBehaviors[laneId].push_back(LongitudinalManeuverType::kFollowing);
        } else {
          lonBehaviors[laneId].push_back(LongitudinalManeuverType::kCrusing);
        }
      } else {
        lonBehaviors[laneId].push_back(LongitudinalManeuverType::kCrusing);
      }
    }
  }

  // Plane for each lan w.r.t. the behaviors defined above.

  JMTTrajectory2d bestTraj;
  double minCost = std::numeric_limits<double>::min();

  for (const auto& [laneName, laneId] : laneIdsForPlanning) {
    if (lonBehaviors.count(laneId) == 0 or latBehaviors.count(laneId) == 0) {
      SPDLOG_DEBUG("Skipping lane {:d}.", laneId);
    }

    // Generate trajectories for ego lane and adjacent lanes
    std::vector<JMTTrajectory1d> lonTrajs;
    std::vector<JMTTrajectory1d> latTrajs;

    SPDLOG_DEBUG("Planning for lane {:d}.", laneId);

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

  return bestTraj;
}

Matrix32d
PolynomialTrajectoryGenerator::_ComputeStartStatePy(const Vehicle& ego,
                                                    const JMTTrajectory2d& prevTraj,
                                                    const Waypoints& prevPath)
{
  py::module pyPTG = py::module::import("pathplanning.ptg");
  py::module pyJson = py::module::import("json");
  py::array_t<double> pyStartState = pyPTG.attr("compute_start_state")(
    pyJson.attr("loads")(ego.Dump().dump()), pyJson.attr("loads")(prevTraj.Dump().dump()), prevPath);
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
  py::module pyPTG = py::module::import("pathplanning.ptg");
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
