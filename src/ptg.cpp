#include "ptg.h"

#include "collision_checker.h"
#include "goalsampler.h"
#include "log.h"
#include "utils.h"

#include <boost/assert.hpp>
#ifdef _OPENMP
#include <omp.h>
#endif

namespace {
using namespace pathplanning;

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
    SPDLOG_DEBUG("Found front id {:2d}, frontSDiff={:7.3f}", frontId, frontSDiff);
  }

  if (backId != -1 and backSDiff < nonEgoSearchRadius) {
    result["back"] = vehicles[backId];
    SPDLOG_DEBUG("Found back id {:2d}, backSDiff={:7.3f}", backId, backSDiff);
  }

  return result;
}

void
_GetOptimalCombination(const std::vector<double>& lonCosts, const std::vector<double>& latCosts, int& lonId, int& latId)
{
  // TODO
  lonId = std::distance(lonCosts.begin(), std::min_element(lonCosts.begin(), lonCosts.end()));
  latId = std::distance(latCosts.begin(), std::min_element(latCosts.begin(), latCosts.end()));
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
                                                      std::vector<JMTTrajectory1d>& trajectories,
                                                      std::vector<double>& costs)
{
  switch (lonBehavior) {
    case LongitudinalManeuverType::kVelocityKeeping:
      _GenerateVelocityKeepingTrajectory(ego, trajectories, costs);
      return;
    case LongitudinalManeuverType::kStopping:
      _GenerateStoppingTrajectory(ego, trajectories, costs);
      return;
    case LongitudinalManeuverType::kFollowing:
      if (vehicle.GetId() != -1) {
        _GenerateVehicleFollowingTrajectory(ego, vehicle, trajectories, costs);
      } else {
        _GenerateVelocityKeepingTrajectory(ego, trajectories, costs);
      }
      return;
  }
}

void
PolynomialTrajectoryGenerator::_GenerateLatTrajectory(const LateralManeuverType& latBehavior,
                                                      const Ego& ego,
                                                      std::vector<JMTTrajectory1d>& trajectories,
                                                      std::vector<double>& costs)
{

  Vector3d egoKinematics = ego.GetKinematics(0.0).block<3, 1>(0, 1);
  double targetD = egoKinematics[0];
  switch (latBehavior) {
    case LateralManeuverType::kLeftLaneChanging:
      targetD = Map::GetLaneCenterD(Map::GetLaneId(egoKinematics[0]) - 1);
      break;
    case LateralManeuverType::kRightLaneChanging:
      targetD = Map::GetLaneCenterD(Map::GetLaneId(egoKinematics[0]) + 1);
      break;
  }

  std::vector<double> TjList = { 3.0, 3.5, 4.0 };
  _SolveFullConstraints1d(
    // Constraints
    egoKinematics[0],
    egoKinematics[1],
    egoKinematics[2],
    0.0,
    targetD,
    0.0,
    // Time
    TjList,
    // S deltas
    { 0.0 },
    // Weights
    1.0,
    1.0,
    10.0,
    100.0,
    1000.0,
    // Outputs
    trajectories,
    costs);
}

void
PolynomialTrajectoryGenerator::_SolveFullConstraints1d(double s0,
                                                       double s0dot,
                                                       double s0ddot,
                                                       double s1,
                                                       double s1dot,
                                                       double s1ddot,
                                                       const std::vector<double>& TjList,
                                                       const std::vector<double>& dsList,
                                                       double kTime,
                                                       double kPos,
                                                       double kVel,
                                                       double kAcc,
                                                       double kJerk,
                                                       std::vector<JMTTrajectory1d>& trajectories,
                                                       std::vector<double>& costs)
{
  Vector6d conditions;
  conditions << s0, s0dot, s0ddot, s1, s1dot, s1ddot;
  for (double ds : dsList) {
    conditions[3] = s1 + ds;
    for (double Tj : TjList) {
      auto traj = JMT::Solve1d(conditions, Tj);
      if (!traj.IsValid(_conf))
        continue;
      trajectories.push_back(traj);
      costs.push_back(traj.ComputeCost(kTime, kPos, kVel, kAcc, kJerk));
    }
  }
}

void
PolynomialTrajectoryGenerator::_GenerateVelocityKeepingTrajectory(const Ego& ego,
                                                                  std::vector<JMTTrajectory1d>& trajectories,
                                                                  std::vector<double>& costs)
{
  std::vector<double> ds1List = { 10.0, 20.0, 30.0, 40.0, 50.0 };
  std::vector<double> TjList = { 3.0, 3.5, 4.0 };

  Vector3d egoKinematics = ego.GetKinematics(0.0).block<3, 1>(0, 0);

  double stepSize = 5.0; // m/s
  for (double i = 1.0; i < 5.01; i += 1.0) {
    if (egoKinematics[1] < i * stepSize) {
      _SolveFullConstraints1d(
        // Constraints
        egoKinematics[0],
        egoKinematics[1],
        egoKinematics[2],
        0.0,
        i * stepSize,
        0.0,
        // Time
        TjList,
        // S deltas
        ds1List,
        // Weights
        1.0,
        1.0,
        1.0,
        1.0,
        1.0,
        // Outputs
        trajectories,
        costs);
    }
  }
}

void
PolynomialTrajectoryGenerator::_GenerateVehicleFollowingTrajectory(const Ego& ego,
                                                                   const Vehicle& vehicle,
                                                                   std::vector<JMTTrajectory1d>& trajectories,
                                                                   std::vector<double>& costs)
{
  // TODO
}

void
PolynomialTrajectoryGenerator::_GenerateStoppingTrajectory(const Ego& ego,
                                                           std::vector<JMTTrajectory1d>& trajectories,
                                                           std::vector<double>& costs)
{
  // TODO
}

Matrix32d
PolynomialTrajectoryGenerator::ComputeStartState(const Vehicle& ego,
                                                 const JMTTrajectory2d& prevTraj,
                                                 const Waypoints& prevPath,
                                                 int numPointsToPreserve)
{
  if (prevPath.empty()) {
    return ego.GetKinematics(0.0);
  }

  double executedTime = (_conf.numPoints - prevPath.size() + numPointsToPreserve) * _conf.simulator.timeStep;
  SPDLOG_DEBUG("executedTime={}", executedTime);
  return prevTraj(executedTime).topRows<3>();
}

JMTTrajectory2d
PolynomialTrajectoryGenerator::GenerataTrajectory(const Ego& ego, const TrackedVehicleMap& trackedVehicleMap)
{
  // For more information on trajectory generation, see
  // "Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame",
  // M. Werling, J. Ziegler, S. Kammel and S. Thrun, ICRA 2010

  auto groupedVehicles = _GroupVehicles(trackedVehicleMap);
  auto egoLandId = Map::GetLaneId(ego.GetKinematics(0.0)(0, 1));

  // Figure out the lanes for planning
  int leftLaneId = egoLandId - 1;
  int rightLaneId = egoLandId + 1;
  std::vector<int> laneIdsForPlanning = { egoLandId };
  if (0 <= leftLaneId and leftLaneId < egoLandId) {
    laneIdsForPlanning.push_back(leftLaneId);
  }
  if (egoLandId < rightLaneId and rightLaneId < Map::NUM_LANES) {
    laneIdsForPlanning.push_back(rightLaneId);
  }

  // Figure out lat behaviors
  std::unordered_map<int, LateralManeuverType> latBehaviors;
  latBehaviors[egoLandId] = LateralManeuverType::kLaneKeeping;
  if (0 <= leftLaneId and leftLaneId < Map::NUM_LANES) {
    latBehaviors[leftLaneId] = LateralManeuverType::kLeftLaneChanging;
  }
  if (0 <= rightLaneId and rightLaneId < Map::NUM_LANES) {
    latBehaviors[rightLaneId] = LateralManeuverType::kRightLaneChanging;
  }

  // Figure out lon behaviors
  std::unordered_map<int, std::vector<LongitudinalManeuverType>> lonBehaviors;
  std::unordered_map<int, Vehicle> leadingVehicles;
  for (const auto& [laneId, vehicles] : groupedVehicles) {
    lonBehaviors[laneId] = { LongitudinalManeuverType::kVelocityKeeping, LongitudinalManeuverType::kStopping };
    auto frontBackVehicles = _GetFrontBackVehiclesPerLane(ego.GetKinematics(0.0), vehicles, 60.0);
    if (frontBackVehicles.count("front")) {
      leadingVehicles[laneId] = frontBackVehicles["front"];
    }
  }

  // Generate trajectories for ego lane and adjacent lanes
  std::vector<JMTTrajectory1d> lonTrajs;
  std::vector<double> lonCosts;

  std::vector<JMTTrajectory1d> latTrajs;
  std::vector<double> latCosts;

  for (const int& laneId : laneIdsForPlanning) {

    // Generate lon trajectories
    for (const auto& behavior : lonBehaviors[laneId]) {
      _GenerateLonTrajectory(behavior, ego, leadingVehicles[laneId], lonTrajs, lonCosts);
    }

    // Generate lat trajectories
    _GenerateLatTrajectory(latBehaviors[laneId], ego, latTrajs, latCosts);
  }

  int lonId = 0, latId = 0;
  _GetOptimalCombination(lonCosts, latCosts, lonId, latId);
  const auto& bestLonTraj = lonTrajs[lonId];
  const auto& bestLatTraj = latTrajs[latId];

  return JMTTrajectory2d(bestLonTraj, bestLatTraj);
}

} // namespace pathplanning
