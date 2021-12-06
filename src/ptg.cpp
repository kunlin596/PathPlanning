#include "ptg.h"

#include "collision_checker.h"
#include "log.h"
#include "utils.h"

#include <boost/assert.hpp>
#include <chrono>
#include <map>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

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
  for (const auto& [vehicleId, vehicle] : trackedVehicleMap) {
    int laneId = Map::GetLaneId(vehicle.GetKinematics(0.0)(0, 1));
    if (laneId < 0) {
      continue;
    }
    if (groupedVehicles.count(laneId) == 0) {
      groupedVehicles[laneId] = std::vector<Vehicle>();
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
void
_FindLeadingFollowingVehicle(const Matrix32d& egoKinematics,
                             const std::vector<Vehicle>& vehicles,
                             double nonEgoSearchRadius,
                             Vehicle& leadingVehicle,
                             double& leadingDistance,
                             Vehicle& followingVehicle,
                             double& followingDistance)
{
  double leadingLonDiff = std::numeric_limits<double>::infinity();
  double followingLonDiff = std::numeric_limits<double>::infinity();

  int leadingId = -1;
  int followingId = -1;

  double egoLon = egoKinematics(0, 0);

  for (size_t i = 0; i < vehicles.size(); ++i) {
    const Vehicle& vehicle = vehicles[i];
    double vehicleLon = vehicle.GetKinematics(0.0)(0, 0);
    double currLeadingLonDiff = vehicleLon - egoLon;
    double currFollowingLonDiff = egoLon - vehicleLon;

    if (0.0 < currLeadingLonDiff and currLeadingLonDiff < leadingLonDiff) {
      leadingLonDiff = currLeadingLonDiff;
      leadingId = i;
    } else if (0.0 < currFollowingLonDiff and currFollowingLonDiff < followingLonDiff) {
      followingLonDiff = currFollowingLonDiff;
      followingId = i;
    }
  }

  if (leadingId != -1 and leadingLonDiff < nonEgoSearchRadius) {
    leadingVehicle = vehicles[leadingId];
    leadingDistance = leadingLonDiff;
    SPDLOG_TRACE("Found leading id {:2d}, leadingLonDiff={:7.3f}", vehicles[leadingId].GetId(), leadingLonDiff);
  }

  if (followingId != -1 and followingLonDiff < nonEgoSearchRadius) {
    followingVehicle = vehicles[followingId];
    followingDistance = followingLonDiff;
    SPDLOG_TRACE("Found following id {:2d}, followingLonDiff={:7.3f}", vehicles[followingId].GetId(), followingLonDiff);
  }
}

void
GetOptimalCombination(const std::vector<JMTTrajectory1d>& lonTrajs,
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

class PolynomialTrajectoryGenerator::Impl
{
public:
  Impl(const Map& map, const Configuration& conf)
    : _map(map)
    , _conf(conf)
  {}

  void GenerateLatTrajectory(const LateralManeuverType& latBehavior,
                             const Ego& ego,
                             std::vector<JMTTrajectory1d>& trajectories) const;

  void GenerateLonTrajectory(const LongitudinalManeuverType& lonBehavior,
                             const Ego& ego,
                             const Vehicle& vehicle,
                             std::vector<JMTTrajectory1d>& trajectories) const;

  void GenerateCruisingTrajectory(const Ego& ego, std::vector<JMTTrajectory1d>& trajectories) const;

  void GenerateVehicleFollowingTrajectory(const Ego& ego,
                                          const Vehicle& vehicle,
                                          std::vector<JMTTrajectory1d>& trajectories) const;

  void GenerateStoppingTrajectory(const Ego& ego, std::vector<JMTTrajectory1d>& trajectories) const;

  JMTTrajectory2d GenerataTrajectoryCpp(const Ego& ego, const TrackedVehicleMap& trackedVehicleMap);

  JMTTrajectory2d GenerataTrajectoryPy(const Ego& ego, const TrackedVehicleMap& trackedVehicleMap);

private:
  const Map& _map;
  const Configuration& _conf;
  int _counter = 0;
};

PolynomialTrajectoryGenerator::PolynomialTrajectoryGenerator(const Map& map, const Configuration& conf)
  : _map(map)
  , _conf(conf)
  , _pImpl(std::make_shared<PolynomialTrajectoryGenerator::Impl>(_map, _conf))
{}

void
PolynomialTrajectoryGenerator::Impl::GenerateLonTrajectory(const LongitudinalManeuverType& lonBehavior,
                                                           const Ego& ego,
                                                           const Vehicle& vehicle,
                                                           std::vector<JMTTrajectory1d>& trajectories) const
{
  switch (lonBehavior) {
    case LongitudinalManeuverType::kCruising:
      GenerateCruisingTrajectory(ego, trajectories);
      return;
    case LongitudinalManeuverType::kStopping:
      GenerateStoppingTrajectory(ego, trajectories);
      return;
    case LongitudinalManeuverType::kFollowing:
      GenerateVehicleFollowingTrajectory(ego, vehicle, trajectories);
      return;
    default:
      return;
  }
}

void
PolynomialTrajectoryGenerator::Impl::GenerateLatTrajectory(const LateralManeuverType& latBehavior,
                                                           const Ego& ego,
                                                           std::vector<JMTTrajectory1d>& trajectories) const
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

  // Recover
  if (egoLatKinematics[0] > _map.GetRoadBoundary()[1]) {
    SPDLOG_WARN("Previous planning went wrong, recover from out of lane from right!");
    targetD = Map::GetLaneCenterD(Map::NUM_LANES - 1);
  } else if (egoLatKinematics[0] < _map.GetRoadBoundary()[0]) {
    SPDLOG_WARN("Previous planning went wrong, recover from out of lane from left!");
    targetD = Map::GetLaneCenterD(0);
  }

  double minTime = _conf.latMinTime;
  double maxTime = _conf.latMaxTime;
  double numTimeSteps = _conf.latNumTimeSteps;
  double timeStep = (maxTime - minTime) / numTimeSteps;

  double minDs = _conf.latMinDs;
  double maxDs = _conf.latMaxDs;
  double numDsSteps = _conf.latNumDsSteps;
  double dsStep = (maxDs - minDs) / numDsSteps;

  std::vector<JMTTrajectory1d> trajs(static_cast<int>(numTimeSteps * numDsSteps));

#ifdef _OPENMP
#pragma omp parallel for
#endif
  for (int i = 0; i < static_cast<int>(numTimeSteps); ++i) {
    double T = minTime + (double)i * timeStep;
    for (int j = 0; j < static_cast<int>(numDsSteps); ++j) {
      double ds = minDs + static_cast<double>(j) * dsStep;
      Vector6d conditions;
      conditions.topRows<3>() = egoLatKinematics;
      conditions.bottomRows<3>() << targetD + ds, 0.0, 0.0;
      auto traj = JMT::Solve1d_6DoF(conditions, T);
      if (traj.Validate(Map::GetRoadBoundary())) {
        traj.ComputeCost(_conf.latTimeWeight, _conf.latPosWeight, _conf.latJerkWeight, _conf.latEfficiencyWeight);
      }
      trajs[i * static_cast<int>(numDsSteps) + j] = traj;
    }
  }

  // NOTE: Hard code the cost threshold for now.
  trajs.erase(
    std::remove_if(trajs.begin(),
                   trajs.end(),
                   [](const JMTTrajectory1d& traj) { return !traj.GetIsValid() and (traj.GetCost() < 100.0); }),
    trajs.end());

  if (!trajs.empty()) {
    std::sort(trajs.begin(), trajs.end(), [](const JMTTrajectory1d& traj1, const JMTTrajectory1d& traj2) {
      return traj1.GetAverageVelocity() > traj2.GetAverageVelocity();
    });
    trajectories.push_back(trajs[0]);
  }
  return;
}

void
PolynomialTrajectoryGenerator::Impl::GenerateCruisingTrajectory(const Ego& ego,
                                                                std::vector<JMTTrajectory1d>& trajectories) const
{
  auto start = std::chrono::steady_clock::now();
  Vector3d egoLonKinematics = ego.GetKinematics(0.0).block<3, 1>(0, 0);
  double minTime = _conf.lonCruiseMinTime;
  double maxTime = _conf.lonCruiseMaxTime;
  double numTimeSteps = _conf.lonCruiseNumTimeSteps;
  double timeStep = (maxTime - minTime) / numTimeSteps;

  double minSddot = _conf.lonCruiseMinSddot;
  double maxSddot = _conf.lonCruiseMaxSddot;
  double numSddotStep = _conf.lonCruiseNumSddotSteps;
  double sddotStep = (maxSddot - minSddot) / numSddotStep;

  int totalCount = static_cast<int>(numTimeSteps * numSddotStep);
  std::vector<JMTTrajectory1d> trajs(totalCount);

#ifdef _OPENMP
#pragma omp parallel for
#endif
  for (int i = 0; i < static_cast<int>(numTimeSteps); ++i) {
    double T = minTime + (double)i * timeStep;
    for (int j = 0; j < static_cast<int>(numSddotStep); ++j) {
      double sddot = minSddot + (double)j * sddotStep;
      Vector5d conditions;
      conditions.topRows<3>() = egoLonKinematics;
      conditions.bottomRows<2>() << std::min(egoLonKinematics[1] + sddot, Mph2Mps(45.0)), 0.0;
      auto traj = JMT::Solve1d_5DoF(conditions, T);
      if (traj.Validate({ egoLonKinematics[0], std::numeric_limits<double>::infinity() })) {
        traj.ComputeCost(_conf.lonCruiseTimeWeight,
                         _conf.lonCruisePosWeight,
                         _conf.lonCruiseJerkWeight,
                         _conf.lonCruiseEfficiencyWeight);
      }
      trajs[i * static_cast<int>(numSddotStep) + j] = traj;
    }
  }

  trajs.erase(
    std::remove_if(trajs.begin(), trajs.end(), [](const JMTTrajectory1d& traj) { return !traj.GetIsValid(); }),
    trajs.end());

  if (!trajs.empty()) {
    std::sort(trajs.begin(), trajs.end(), [](const JMTTrajectory1d& traj1, const JMTTrajectory1d& traj2) {
      return traj1.GetAverageVelocity() > traj2.GetAverageVelocity();
    });
    trajectories.push_back(trajs[0]);
  }

  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsedTime = end - start;
  if (elapsedTime.count() > 0.1) {
    SPDLOG_WARN("Planned for {:d} condition combinations, valid size is {:d}, elapsed time {:.3f} sec.",
                totalCount,
                trajs.size(),
                elapsedTime.count());
  }
}

void
PolynomialTrajectoryGenerator::Impl::GenerateVehicleFollowingTrajectory(
  const Ego& ego,
  const Vehicle& vehicle,
  std::vector<JMTTrajectory1d>& trajectories) const
{
  auto start = std::chrono::steady_clock::now();

  Vector3d egoLonKinematics = ego.GetKinematics(0.0).block<3, 1>(0, 0);
  Vector3d vehicleLonKinematics = vehicle.GetKinematics(0.0).block<3, 1>(0, 0);
  double minTime = _conf.lonFollowingMinTime;
  double maxTime = _conf.lonFollowingMaxTime;
  double numTimeSteps = _conf.lonFollowingNumTimeSteps;
  double timeStep = (maxTime - minTime) / numTimeSteps;

  double tau = _conf.lonFollowingTau;
  double offset = _conf.lonFollowingLonOffset;

  std::vector<JMTTrajectory1d> trajs(numTimeSteps);
#ifdef _OPENMP
#pragma omp parallel for
#endif
  for (int i = 0; i < static_cast<int>(numTimeSteps); ++i) {
    double T = minTime + timeStep * static_cast<double>(i);
    Vector3d leadingLonKinematics = vehicle.GetKinematics(T).col(0);
    Vector6d conditions;
    conditions.block<3, 1>(0, 0) = egoLonKinematics;
    // clang-format off
    conditions.block<3, 1>(3, 0) <<
      leadingLonKinematics[0] - tau * leadingLonKinematics[1] - offset,
      std::max(leadingLonKinematics[1] - tau * leadingLonKinematics[2], Mph2Mps(45.0)),
      leadingLonKinematics[2];
    // clang-format on
    auto traj = JMT::Solve1d_6DoF(conditions, T);
    if (traj.Validate({ egoLonKinematics[0], std::numeric_limits<double>::infinity() })) {
      traj.ComputeCost(_conf.lonFollowingTimeWeight,
                       _conf.lonFollowingPosWeight,
                       _conf.lonFollowingJerkWeight,
                       _conf.lonFollowingEfficiencyWeight);
    }
    trajs[i] = traj;
  }

  trajs.erase(
    std::remove_if(trajs.begin(), trajs.end(), [](const JMTTrajectory1d& traj) { return !traj.GetIsValid(); }),
    trajs.end());

  if (!trajs.empty()) {
    std::sort(trajs.begin(), trajs.end(), [](const JMTTrajectory1d& traj1, const JMTTrajectory1d& traj2) {
      return traj1.GetAverageVelocity() > traj2.GetAverageVelocity();
    });
    trajectories.push_back(trajs[0]);
  }

  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsedTime = end - start;
  if (elapsedTime.count() > 0.1) {
    SPDLOG_INFO("Planned for {:d} condition combinations, valid size is {:d}, elapsed time {:.3f} sec.",
                numTimeSteps,
                trajs.size(),
                elapsedTime.count());
  }
}

void
PolynomialTrajectoryGenerator::Impl::GenerateStoppingTrajectory(const Ego& ego,
                                                                std::vector<JMTTrajectory1d>& trajectories) const
{
  auto start = std::chrono::steady_clock::now();

  Vector3d egoLonKinematics = ego.GetKinematics(0.0).block<3, 1>(0, 0);

  double minTime = _conf.lonStoppingMinTime;
  double maxTime = _conf.lonStoppingMaxTime;
  double numTimeSteps = _conf.lonStoppingNumTimeSteps;
  double timeStep = (maxTime - minTime) / numTimeSteps;

  double minSddot = _conf.lonStoppingMinSddot;
  double maxSddot = _conf.lonStoppingMaxSddot;
  double numSddotStep = _conf.lonStoppingNumSddotSteps;
  double sddotStep = (maxSddot - minSddot) / numSddotStep;

  std::vector<JMTTrajectory1d> trajs(static_cast<int>(numTimeSteps * numSddotStep));

#ifdef _OPENMP
#pragma omp parallel for
#endif
  for (int i = 0; i < static_cast<int>(numTimeSteps); ++i) {
    double T = minTime + timeStep * static_cast<double>(i);
    for (int j = 0; j < static_cast<int>(numSddotStep); ++j) {
      double sddot = minSddot + sddotStep * static_cast<double>(j);
      Vector5d conditions;
      conditions.block<3, 1>(0, 0) = egoLonKinematics;
      conditions.block<2, 1>(3, 0) << egoLonKinematics[1] + sddot, 0.0;
      auto traj = JMT::Solve1d_5DoF(conditions, T);
      if (traj.Validate({ egoLonKinematics[0], std::numeric_limits<double>::infinity() })) {
        traj.ComputeCost(_conf.lonStoppingTimeWeight,
                         _conf.lonStoppingPosWeight,
                         _conf.lonStoppingJerkWeight,
                         _conf.lonStoppingEfficiencyWeight);
      }
      trajs[i * static_cast<int>(numSddotStep) + j] = traj;
    }
  }

  trajs.erase(
    std::remove_if(trajs.begin(), trajs.end(), [](const JMTTrajectory1d& traj) { return !traj.GetIsValid(); }),
    trajs.end());

  if (!trajs.empty()) {
    std::sort(trajs.begin(), trajs.end(), [](const JMTTrajectory1d& traj1, const JMTTrajectory1d& traj2) {
      return traj1.GetAverageVelocity() > traj2.GetAverageVelocity();
    });
    trajectories.push_back(trajs[0]);
  }

  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsedTime = end - start;
  if (elapsedTime.count() > 0.1) {
    SPDLOG_INFO("Planned for {:d} condition combinations, valid size is {:d}, elapsed time {:.3f} sec.",
                static_cast<int>(numTimeSteps * numSddotStep),
                trajs.size(),
                elapsedTime.count());
  }
}

JMTTrajectory2d
PolynomialTrajectoryGenerator::Impl::GenerataTrajectoryCpp(const Ego& ego, const TrackedVehicleMap& trackedVehicleMap)
{
  // For more information on trajectory generation, see
  // "Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame",
  // M. Werling, J. Ziegler, S. Kammel and S. Thrun, ICRA 2010

  // The trajectory generation strategy is described as follows.
  //
  // For each possible lane, where will be,
  // {
  //   [
  //     // lane 0
  //     {
  //       "lonBehaviors": {
  //         "kCruising",
  //         "kStopping",
  //         ...
  //       },
  //       "latBehaviors": [
  //         "kLaneKeeping",
  //         "kRightLaneChanging"
  //       ]
  //     },
  //     // lane 1
  //     {
  //       "lonBehaviors": {
  //         "kCruising",
  //         "kStopping",
  //         ...
  //       },
  //       "latBehaviors": [
  //         "kLaneKeeping",
  //         "kRightLaneChanging",
  //         "kRightLaneChanging"
  //       ]
  //     },
  //     ...
  //   ]
  // }
  //
  // Then for each lane, the optimal combination of longitudinal and lateral 1D trajectories will be computed to find
  // the feasible and best 2D plan for that lane.
  //
  // Finally we compare all 2D trajectories for each lane with each other and determine the final winner.

  using std::map;
  using std::string;
  using std::unordered_map;
  using std::vector;

  SPDLOG_INFO("-------------------- Planning starting {:d} --------------------", ++_counter);
  auto egoLaneId = Map::GetLaneId(ego.GetKinematics(0.0)(0, 1));

  // Figure out the lanes for planning
  int leftLaneId = egoLaneId - 1;
  int rightLaneId = egoLaneId + 1;

  // Lane ids are determined by lateral planning decisions, since longitudinal planning will happen anyways.
  // Depending on which lane ego is on, we can decide which lane to plan.
  map<LaneType, int> laneIdsForPlanning;
  if (0 <= leftLaneId and leftLaneId < egoLaneId) {
    laneIdsForPlanning[LaneType::kLeft] = leftLaneId;
  }
  laneIdsForPlanning[LaneType::kEgo] = egoLaneId;
  if (egoLaneId < rightLaneId and rightLaneId < Map::NUM_LANES) {
    laneIdsForPlanning[LaneType::kRight] = rightLaneId;
  }

  // Figure out lateral behaviors w.r.t. lane ids for planning
  map<int, LateralManeuverType> latBehaviors;
  if (laneIdsForPlanning.count(LaneType::kLeft)) {
    latBehaviors[leftLaneId] = LateralManeuverType::kLeftLaneChanging;
  }
  latBehaviors[egoLaneId] = LateralManeuverType::kLaneKeeping;
  if (laneIdsForPlanning.count(LaneType::kRight)) {
    latBehaviors[rightLaneId] = LateralManeuverType::kRightLaneChanging;
  }

  // Group vehicles for longitudinal planning.
  unordered_map<int, std::vector<Vehicle>> groupedVehicles = _GroupVehicles(trackedVehicleMap);

  // Cache the leading vehicles when checking for longitudinal behaviors.
  unordered_map<int, Vehicle> leadingVehicles;

  // Figure out longitudinal behaviors w.r.t. the traffic conditions on that lane.
  map<int, vector<LongitudinalManeuverType>> lonBehaviors;

  double maxLaneChangingTriggerDistance = _conf.maxLaneChangingTriggerDistance;
  double maxStoppingTriggerDistance = _conf.maxStoppingTriggerDistance;
  double maxFollowTriggerDistance = _conf.maxFollowTriggerDistance;

  SPDLOG_INFO("  Compute longitudinal behavior.");
  for (const auto& [laneName, laneId] : laneIdsForPlanning) {
    if (latBehaviors.count(laneId) == 0) {
      continue;
    }

    lonBehaviors[laneId] = vector<LongitudinalManeuverType>();

    // If there are vehicles, find the leading vehicle and plan accordingly.
    if (groupedVehicles.count(laneId) and !groupedVehicles[laneId].empty()) {
      const auto& vehicles = groupedVehicles[laneId];

      Vehicle leadingVehicle;
      double leadingDistance = std::numeric_limits<double>::quiet_NaN();
      Vehicle followingVehicle;
      double followingDistance = std::numeric_limits<double>::quiet_NaN();
      _FindLeadingFollowingVehicle(ego.GetKinematics(0.0),
                                   vehicles,
                                   _conf.nonEgoSearchRadius,
                                   leadingVehicle,
                                   leadingDistance,
                                   followingVehicle,
                                   followingDistance);

      // TODO: Take following vehicle in to account and implement merging behavior.
      // NOTE: The distance values for determine the behavior below is purely heuristic.
      if (!std::isnan(leadingDistance)) {
        if (leadingDistance < maxLaneChangingTriggerDistance) {
          // Too close, do nothing
        } else if (maxLaneChangingTriggerDistance < leadingDistance and leadingDistance < maxStoppingTriggerDistance) {
          // Leading vehicle is too close so need to start stopping now!
          lonBehaviors[laneId].push_back(LongitudinalManeuverType::kStopping);
          SPDLOG_INFO("   - On lane {:d}, {:d} has a distance {:.3f} less than {:.3f}, added {:s}.",
                      laneId,
                      leadingVehicle.GetId(),
                      leadingDistance,
                      maxStoppingTriggerDistance,
                      LongitudinalManeuverType::kStopping);
        } else if (maxStoppingTriggerDistance < leadingDistance and leadingDistance < maxFollowTriggerDistance) {
          // Leading vehicle is within a reasonable distance to be followed.
          lonBehaviors[laneId].push_back(LongitudinalManeuverType::kFollowing);
          leadingVehicles[laneId] = leadingVehicle;
          SPDLOG_INFO("   - On lane {:d}, {:d} has a distance {:.3f} less than {:.3f}, added {:s}.",
                      laneId,
                      leadingVehicle.GetId(),
                      leadingDistance,
                      maxFollowTriggerDistance,
                      LongitudinalManeuverType::kFollowing);
        } else {
          // Leading vehicle is too far from a possible following distance.
          SPDLOG_INFO("   - On lane {:d}, {:d} has a distance {:.3f} more than {:.3f}, added {:s}.",
                      laneId,
                      leadingVehicle.GetId(),
                      leadingDistance,
                      maxFollowTriggerDistance,
                      LongitudinalManeuverType::kCruising);
          lonBehaviors[laneId].push_back(LongitudinalManeuverType::kCruising);
        }
      } else {
        // No leading vehicle in the search distance.
        SPDLOG_INFO("   - On lane {:d}, there is no vehicles within search range, added {:s}.",
                    laneId,
                    LongitudinalManeuverType::kCruising);
        lonBehaviors[laneId].push_back(LongitudinalManeuverType::kCruising);
      }
    }

    else {
      // No vehicles, just plan for cruising.
      SPDLOG_INFO(
        "   - On lane {:d} there is no vehicles at all, added {:s}.", laneId, LongitudinalManeuverType::kCruising);
      lonBehaviors[laneId].push_back(LongitudinalManeuverType::kCruising);
    }
  }

  // Plan for JMT for each lane w.r.t. the behaviors determined above.
  JMTTrajectory2d bestTraj;
  double minCost = std::numeric_limits<double>::quiet_NaN();
  for (const auto& [laneName, laneId] : laneIdsForPlanning) {
    if (lonBehaviors.count(laneId) == 0 or latBehaviors.count(laneId) == 0) {
      SPDLOG_INFO("  Skipping lane {:d}.", laneId);
      continue;
    }

    if (lonBehaviors[laneId].empty()) {
      SPDLOG_INFO("  Skipping lane {:d}.", laneId);
      continue;
    }

    // Generate 1D trajectories for the current lane.
    vector<JMTTrajectory1d> lonTrajs;
    vector<JMTTrajectory1d> latTrajs;

    SPDLOG_INFO("  Planning for lane {:d}.", laneId);

    // Generate longitudinal trajectories
    for (const auto& behavior : lonBehaviors[laneId]) {
      SPDLOG_INFO("   - Planning for {:s}.", behavior);
      GenerateLonTrajectory(behavior, ego, leadingVehicles[laneId], lonTrajs);
    }

    // Generate lateral trajectories
    GenerateLatTrajectory(latBehaviors[laneId], ego, latTrajs);
    SPDLOG_INFO("   - Planning for {:s}.", latBehaviors[laneId]);

    if (lonTrajs.empty() or latTrajs.empty()) {
      SPDLOG_INFO(
        "   - Planning failed for lane {:d}, #lonTrajs={:d}, #latTrajs={:d}", laneId, lonTrajs.size(), latTrajs.size());
      continue;
    }

    // Figure out optimal combination
    int lonId = -1, latId = -1;
    double cost;
    GetOptimalCombination(lonTrajs, latTrajs, lonId, latId, cost);

    if (lonId == -1 or latId == -1) {
      SPDLOG_INFO("   - Planning failed for lane {:d}, cannot find optimal combination, lonId={:d}, latId={:d}",
                  laneId,
                  lonId,
                  latId);
      continue;
    }

    const auto& bestLonTraj = lonTrajs[lonId];
    const auto& bestLatTraj = latTrajs[latId];
    auto traj = JMTTrajectory2d(bestLonTraj, bestLatTraj);

    SPDLOG_INFO("   - Best lon cost {:.3f}, lat cost {:.3f}", bestLonTraj.GetCost(), bestLatTraj.GetCost());

    traj.Validate(_map, _conf);
    if (!traj.GetIsValid()) {
      SPDLOG_INFO("   - Planning failed for lane {:d}, best trajectory for lane is not valid.", laneId);
      continue;
    }

    auto [collidedVehicleId, distance] = CollisionChecker::IsInCollision(traj, groupedVehicles[laneId], _conf);
    if (collidedVehicleId != -1) {
      const Vehicle& collidedVehicle = trackedVehicleMap.at(collidedVehicleId);
      int collisionLaneId = Map::GetLaneId(collidedVehicle.GetKinematics(0.0)(0, 1));
      SPDLOG_INFO("   - Planning failed for lane {:d}, trajectory is colliding with collidedVehicleId={:d}, "
                  "distance={:7.3f}, collisionLaneId={:d}, collision lonlat=[{}], ego lonlat=[{}]",
                  laneId,
                  collidedVehicleId,
                  distance,
                  collisionLaneId,
                  collidedVehicle.GetKinematics(0.0).topRows<1>(),
                  ego.GetKinematics(0.0).topRows<1>());
      continue;
    }

    SPDLOG_INFO("   * Planning completed for lane {:d}, cost={:7.3f}", laneId, cost);

    if (std::isnan(minCost) or cost < minCost) {
      minCost = cost;
      bestTraj = traj;
    }
  }

  if (std::isnan(minCost)) {
    SPDLOG_ERROR("  Planning failed for all lanes.");
  } else {
    SPDLOG_INFO("  Planning completed, minCost={:7.3f}", minCost);
  }

  return bestTraj;
}

JMTTrajectory2d
PolynomialTrajectoryGenerator::Impl::GenerataTrajectoryPy(const Ego& ego, const TrackedVehicleMap& trackedVehicleMap)
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

JMTTrajectory2d
PolynomialTrajectoryGenerator::GenerataTrajectory(const Ego& ego, const TrackedVehicleMap& trackedVehicleMap)
{
  if (_conf.mode == "python") {
    return _pImpl->GenerataTrajectoryPy(ego, trackedVehicleMap);
  } else if (_conf.mode == "cpp") {
    return _pImpl->GenerataTrajectoryCpp(ego, trackedVehicleMap);
  }

  throw std::runtime_error(
    fmt::format("Planner mode {} is not supported, choose from [`python` and `cpp`].", _conf.mode));
}

std::ostream&
operator<<(std::ostream& out, const LongitudinalManeuverType& type)
{
  switch (type) {
    case LongitudinalManeuverType::kCruising:
      out << "Cruising (0)";
      return out;
    case LongitudinalManeuverType::kFollowing:
      out << "Following (1)";
      return out;
    case LongitudinalManeuverType::kStopping:
      out << "Stopping (2)";
      return out;
    default:
      return out;
  }
}

std::ostream&
operator<<(std::ostream& out, const LateralManeuverType& type)
{
  switch (type) {
    case LateralManeuverType::kLaneKeeping:
      out << "LaneKeeping (0)";
      return out;
    case LateralManeuverType::kLeftLaneChanging:
      out << "LeftLaneChanging (1)";
      return out;
    case LateralManeuverType::kRightLaneChanging:
      out << "RightLaneChanging (2)";
      return out;
    default:
      return out;
  }
}

} // namespace pathplanning
