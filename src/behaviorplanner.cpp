#include "behaviorplanner.h"

namespace {
using namespace pathplanning;

/**
 * @brief      Find front and back vehicle in current lane
 *
 * @param[in]  ego                The ego
 * @param[in]  trackedVehicleMap  The tracked vehicle map
 *
 * @return     Vehicle map
 */
std::unordered_map<std::string, Vehicle>
_GetFrontBackVehicles(const Vehicle& ego,
                      const TrackedVehicleMap& trackedVehicleMap,
                      int targetLaneId = -1)
{
  const auto& egoConf = ego.GetConfiguration();
  int currentLaneId =
    targetLaneId == -1 ? Map::GetLaneId(egoConf.dPos) : targetLaneId;
  double frontSDiff = std::numeric_limits<double>::max();
  double backSDiff = std::numeric_limits<double>::max();
  int frontId = -1;
  int backId = -1;

  for (const auto& vehicleData : trackedVehicleMap) {
    const auto& vehicleConf = vehicleData.second.GetConfiguration();

    int carLaneId = Map::GetLaneId(vehicleConf.dPos);
    if (carLaneId < 0) {
      continue;
    }

    SPDLOG_ERROR("currentLaneId={}, carLaneId={}", currentLaneId, carLaneId);
    if (carLaneId == currentLaneId) {
      double currFrontSDiff = vehicleConf.sPos - egoConf.sPos;
      double currBackSDiff = egoConf.sPos - vehicleConf.sPos;
      if (0.0 < currFrontSDiff and currFrontSDiff < frontSDiff) {
        frontSDiff = currFrontSDiff;
        frontId = vehicleData.first;

      } else if (0.0 < currBackSDiff and currBackSDiff < backSDiff) {
        backSDiff = currBackSDiff;
        backId = vehicleData.first;
      }
    }
  }
  std::unordered_map<std::string, Vehicle> result;
  if (frontId != -1 and frontSDiff < Configuration::NONEGO_SEARCH_RADIUS) {
    result["front"] = trackedVehicleMap.at(frontId);
    SPDLOG_INFO(
      "Found front id {:2d}, frontSDiff={:7.3f}", frontId, frontSDiff);
  }
  if (backId != -1 and backSDiff < Configuration::NONEGO_SEARCH_RADIUS) {
    result["back"] = trackedVehicleMap.at(backId);
  }
  return result;
}

Vehicle
_GenerateLKProposal(const Vehicle& ego,
                    const TrackedVehicleMap& trackedVehicleMap)
{
  auto frontBackVehicles = _GetFrontBackVehicles(ego, trackedVehicleMap);
  const auto& egoConf = ego.GetConfiguration();
  if (frontBackVehicles.count("front")) {
    auto vehicleConf = frontBackVehicles["front"].GetConfiguration();
    // TODO: Properly set s offset w.r.t. leading car
    return Vehicle(ego.GetId(),
                   VehicleConfiguration(vehicleConf.sPos - 20.0,
                                        vehicleConf.sVel,
                                        0.0,
                                        egoConf.dPos,
                                        0.0,
                                        0.0));
  }
  return Vehicle(
    ego.GetId(),
    VehicleConfiguration(egoConf.sPos + 30.0,
                         5.0,
                         0.0,
                         Map::GetLaneCenterD(Map::GetLaneId(egoConf.dPos)),
                         0.0,
                         0.0));
}

Vehicle
_GenerateLCProposal(const Vehicle& ego,
                    const TrackedVehicleMap& trackedVehicleMap,
                    int laneOffset = -1)
{
  const auto& egoConf = ego.GetConfiguration();
  int currentLaneId = Map::GetLaneId(egoConf.dPos);

  if (currentLaneId + laneOffset < 0) {
    return ego;
  }

  int targetLaneId = currentLaneId + laneOffset;
  auto frontBackVehicles =
    _GetFrontBackVehicles(ego, trackedVehicleMap, targetLaneId);

  if (frontBackVehicles.count("front")) {
    auto vehicleConf = frontBackVehicles["front"].GetConfiguration();
    // TODO: Properly set s offset w.r.t. leading car
    return Vehicle(ego.GetId(),
                   VehicleConfiguration(vehicleConf.sPos + 30.0,
                                        5.0,
                                        0.0,
                                        Map::GetLaneCenterD(targetLaneId),
                                        0.0,
                                        0.0));
  }
  return Vehicle(ego.GetId(),
                 VehicleConfiguration(egoConf.sPos + 30.0,
                                      5.0,
                                      0.0,
                                      Map::GetLaneCenterD(targetLaneId),
                                      0.0,
                                      0.0));
}

Vehicle
_GenerateLLCProposal(const Vehicle& ego,
                     const TrackedVehicleMap& trackedVehicleMap)
{
  return _GenerateLCProposal(ego, trackedVehicleMap, -1);
}

Vehicle
_GenerateRLCProposal(const Vehicle& ego,
                     const TrackedVehicleMap& trackedVehicleMap)
{
  return _GenerateLCProposal(ego, trackedVehicleMap, 1);
}

} // namespace

namespace pathplanning {
BehaviorPlanner::BehaviorPlanner(const Map::ConstPtr& pMap)
  : _pMap(pMap)
{
  costs::CostWeightMapping mapping;
  mapping[costs::CostType::kTimeDiff] = 1.0;
  mapping[costs::CostType::kSDiff] = 1.0;
  mapping[costs::CostType::kDDiff] = 1.0;
  mapping[costs::CostType::kCollision] = 1e6;
  mapping[costs::CostType::kBuffer] = 100.0;
  mapping[costs::CostType::kStaysOnRoad] = 1.0;
  mapping[costs::CostType::kExceedsSpeedLimit] = 1.0;
  mapping[costs::CostType::kEfficiency] = 1.0;
  mapping[costs::CostType::kTotalAccel] = 1.0;
  mapping[costs::CostType::kMaxAccel] = 1.0;
  mapping[costs::CostType::kTotalJerk] = 1.0;
  mapping[costs::CostType::kMaxJerk] = 1.0;

  _pEvaluator = std::make_unique<JMTTrajectoryEvaluator>(mapping);
}

std::vector<BehaviorState>
BehaviorPlanner::GetSuccessorStates() const
{
  std::vector<BehaviorState> states = { BehaviorState::kLaneKeeping };
  // FIXME: Add LCP
  if (_currState == BehaviorState::kLaneKeeping) {
    // states.push_back(BehaviorState::kLeftLaneChangePreparation);
    // states.push_back(BehaviorState::kRightLaneChangePreparation);
    states.push_back(BehaviorState::kLeftLaneChange);
    states.push_back(BehaviorState::kRightLaneChange);
  } else if (_currState == BehaviorState::kLeftLaneChangePreparation) {
    // states.push_back(BehaviorState::kLeftLaneChangePreparation);
    states.push_back(BehaviorState::kLeftLaneChange);
  } else if (_currState == BehaviorState::kRightLaneChangePreparation) {
    // states.push_back(BehaviorState::kRightLaneChangePreparation);
    states.push_back(BehaviorState::kRightLaneChange);
  }
  return states;
}

Vehicle
BehaviorPlanner::GenerateProposal(
  const Vehicle& ego,
  const Waypoints& prevPath,
  const Waypoint& endPrevPathSD,
  const std::vector<BehaviorState> successorStates,
  const TrackedVehicleMap& trackedVehicleMap) const
{
  const auto& egoConf = ego.GetConfiguration();
  VehicleConfiguration bestProposal;
  double minCost = std::numeric_limits<double>::max();
  BehaviorState bestState;

  for (const auto& state : successorStates) {
    Vehicle goalState;
    double time = 2.0;
    if (state == BehaviorState::kLaneKeeping) {
      goalState = _GenerateLKProposal(ego, trackedVehicleMap);
    } else if (state == BehaviorState::kLeftLaneChange) {
      goalState = _GenerateLLCProposal(ego, trackedVehicleMap);
    } else if (state == BehaviorState::kRightLaneChange) {
      goalState = _GenerateRLCProposal(ego, trackedVehicleMap);
    } else {
      // fallback case
      goalState = _GenerateLKProposal(ego, trackedVehicleMap);
    }

    const auto& goalConf = goalState.GetConfiguration();

    // HACK
    if (goalConf.dPos < 1e-6 or goalConf.dPos > (12.0 - 1e-6)) {
      continue;
    }

    if (goalConf.sPos < egoConf.sPos + 10.0) {
      continue;
    }

    auto traj = JMT::ComputeTrajectory(egoConf, goalConf, time);
    double cost =
      _pEvaluator->Evaluate(traj, goalConf, time, trackedVehicleMap);

    SPDLOG_ERROR("goalConf={}, Cost {:s}: {:7.3}", goalConf, state, cost);
    if (cost < minCost) {
      minCost = cost;
      bestProposal = goalConf;
      bestState = state;
      SPDLOG_ERROR("Update best {:s}: {:7.3}", bestState, minCost);
    }
  }

  SPDLOG_INFO("bestProposal={}, bestState={:s}", bestProposal, bestState);

  return Vehicle(ego.GetId(), bestProposal);
}

} // namespace pathplanning
