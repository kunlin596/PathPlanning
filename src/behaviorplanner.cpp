#include "behaviorplanner.h"
#include <memory>

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
                      int targetLaneId = -1,
                      double nonEgoSearchRadius = 30.0)
{
  const auto& egoConf = ego.GetConfiguration();
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

    if (carLaneId == targetLaneId) {
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

  if (frontId != -1 and frontSDiff < nonEgoSearchRadius) {
    result["front"] = trackedVehicleMap.at(frontId);
    SPDLOG_DEBUG(
      "Found front id {:2d}, frontSDiff={:7.3f}", frontId, frontSDiff);
  }
  if (backId != -1 and backSDiff < nonEgoSearchRadius) {
    result["back"] = trackedVehicleMap.at(backId);
    SPDLOG_DEBUG("Found back id {:2d}, backSDiff={:7.3f}", backId, backSDiff);
  }
  return result;
}

Vehicle
_GenerateLKProposal(const Vehicle& ego,
                    const TrackedVehicleMap& trackedVehicleMap,
                    const BehaviorPlanner::Options& options)
{
  const auto& egoConf = ego.GetConfiguration();
  auto frontBackVehicles = _GetFrontBackVehicles(ego,
                                                 trackedVehicleMap,
                                                 Map::GetLaneId(egoConf.dPos),
                                                 options.nonEgoSearchRadius);
  if (frontBackVehicles.count("front")) {
    auto fronVehicleConf =
      frontBackVehicles["front"].GetConfiguration(options.timeHorizon);

    SPDLOG_INFO("Tracing {:2d}", frontBackVehicles["front"].GetId());

    // TODO: Properly set s offset w.r.t. leading car
    return Vehicle(ego.GetId(),
                   VehicleConfiguration(fronVehicleConf.sPos - 20.0,
                                        fronVehicleConf.sVel,
                                        0.0,
                                        egoConf.dPos,
                                        0.0,
                                        0.0));
  }
  return Vehicle(
    ego.GetId(),
    VehicleConfiguration(egoConf.sPos + 10.0,
                         Mph2Mps(49.5),
                         0.0,
                         Map::GetLaneCenterD(Map::GetLaneId(egoConf.dPos)),
                         0.0,
                         0.0));
}

Vehicle
_GenerateLCPProposal(const Vehicle& ego,
                     const TrackedVehicleMap& trackedVehicleMap,
                     const BehaviorPlanner::Options& options,
                     int laneOffset = -1)
{
  return Vehicle();
}

Vehicle
_GenerateLLCPProposal(const Vehicle& ego,
                      const TrackedVehicleMap& trackedVehicleMap,
                      const BehaviorPlanner::Options& options)
{
  return _GenerateLCPProposal(ego, trackedVehicleMap, options, -1);
}

Vehicle
_GenerateRLCPProposal(const Vehicle& ego,
                      const TrackedVehicleMap& trackedVehicleMap,
                      const BehaviorPlanner::Options& options)
{
  return _GenerateLCPProposal(ego, trackedVehicleMap, options, -1);
}

Vehicle
_GenerateLCProposal(const Vehicle& ego,
                    const TrackedVehicleMap& trackedVehicleMap,
                    const BehaviorPlanner::Options& options,
                    int laneOffset = -1)
{
  const auto& egoConf = ego.GetConfiguration();
  int currentLaneId = Map::GetLaneId(egoConf.dPos);
  int targetLaneId = currentLaneId + laneOffset;
  auto frontBackVehicles =
    _GetFrontBackVehicles(ego, trackedVehicleMap, targetLaneId);

  if (frontBackVehicles.count("front")) {
    auto vehicleConf =
      frontBackVehicles["front"].GetConfiguration(options.timeHorizon);
    SPDLOG_INFO("Chang lane to {} to follow {}", targetLaneId, vehicleConf);
    return Vehicle(ego.GetId(),
                   VehicleConfiguration(vehicleConf.sPos - 10.0,
                                        vehicleConf.sVel,
                                        0.0,
                                        Map::GetLaneCenterD(targetLaneId),
                                        0.0,
                                        0.0));
  }

  SPDLOG_INFO("Chang lane to {}", targetLaneId);
  return Vehicle(ego.GetId(),
                 VehicleConfiguration(egoConf.sPos + 10.0,
                                      Mph2Mps(49.5),
                                      0.0,
                                      Map::GetLaneCenterD(targetLaneId),
                                      0.0,
                                      0.0));
}

Vehicle
_GenerateLLCProposal(const Vehicle& ego,
                     const TrackedVehicleMap& trackedVehicleMap,
                     const BehaviorPlanner::Options& options)
{
  return _GenerateLCProposal(ego, trackedVehicleMap, options, -1);
}

Vehicle
_GenerateRLCProposal(const Vehicle& ego,
                     const TrackedVehicleMap& trackedVehicleMap,
                     const BehaviorPlanner::Options& options)
{
  return _GenerateLCProposal(ego, trackedVehicleMap, options, -1);
}

} // namespace

namespace pathplanning {
BehaviorPlanner::BehaviorPlanner(const Map::ConstPtr& pMap,
                                 const Options& options)
  : _pMap(pMap)
  , _options(options)
{
  _pEvaluator =
    std::make_unique<JMTTrajectoryEvaluator>(_options.trajEvaluationOptions);
}

std::vector<BehaviorState>
BehaviorPlanner::GetSuccessorStates() const
{
  std::vector<BehaviorState> states = { BehaviorState::kLaneKeeping };
  // if (_currState == BehaviorState::kLaneKeeping) {
  //   // states.push_back(BehaviorState::kLeftLaneChangePreparation);
  //   // states.push_back(BehaviorState::kRightLaneChangePreparation);
  //   states.push_back(BehaviorState::kLeftLaneChange);
  //   states.push_back(BehaviorState::kRightLaneChange);
  // } else if (_currState == BehaviorState::kLeftLaneChangePreparation) {
  //   // states.push_back(BehaviorState::kLeftLaneChangePreparation);
  //   states.push_back(BehaviorState::kLeftLaneChange);
  // } else if (_currState == BehaviorState::kRightLaneChangePreparation) {
  //   // states.push_back(BehaviorState::kRightLaneChangePreparation);
  //   states.push_back(BehaviorState::kRightLaneChange);
  // }
  return states;
}

Vehicle
BehaviorPlanner::GenerateProposal(
  const Vehicle& ego,
  const Waypoints& prevPath,
  const Waypoint& endPrevPathSD,
  const std::vector<BehaviorState> successorStates,
  const TrackedVehicleMap& trackedVehicleMap)
{
  const auto& egoConf = ego.GetConfiguration();
  VehicleConfiguration bestProposal;
  double minCost = std::numeric_limits<double>::max();
  BehaviorState bestState;

  for (const auto& state : successorStates) {
    SPDLOG_DEBUG("Checking {}", state);
    Vehicle goalState;
    if (state == BehaviorState::kLaneKeeping) {
      goalState = _GenerateLKProposal(ego, trackedVehicleMap, _options);
    } else if (state == BehaviorState::kLeftLaneChange) {
      goalState = _GenerateLLCProposal(ego, trackedVehicleMap, _options);
    } else if (state == BehaviorState::kRightLaneChange) {
      goalState = _GenerateRLCProposal(ego, trackedVehicleMap, _options);
    } else if (state == BehaviorState::kLeftLaneChangePreparation) {

    } else {
      // fallback case
      goalState = _GenerateLKProposal(ego, trackedVehicleMap, _options);
    }

    const auto& goalConf = goalState.GetConfiguration();

    auto traj = JMT::ComputeTrajectory(egoConf, goalConf, _options.timeHorizon);
    double cost = _pEvaluator->Evaluate(
      traj, goalConf, _options.timeHorizon, trackedVehicleMap);

    SPDLOG_DEBUG("  Current cost {:s}: {:7.3}", state, cost);
    if (cost < minCost) {
      minCost = cost;
      bestProposal = goalConf;
      bestState = state;
      SPDLOG_INFO("  Update best {:s}: {:7.3}", bestState, minCost);
    }
  }

  SPDLOG_INFO("bestProposal={}, bestState={:s}", bestProposal, bestState);

  _currState = bestState;
  return Vehicle(ego.GetId(), bestProposal);
}

std::ostream&
operator<<(std::ostream& out, const BehaviorState& type)
{
  using namespace pathplanning;
  switch (type) {
    case BehaviorState::kStart:
      return out << "Start";
    case BehaviorState::kStop:
      return out << "Stop";
    case BehaviorState::kConstSpeed:
      return out << "ConstSpeed";
    case BehaviorState::kLaneKeeping:
      return out << "LaneKeeping";
    case BehaviorState::kLeftLaneChangePreparation:
      return out << "LeftLaneChangePreparation";
    case BehaviorState::kLeftLaneChange:
      return out << "LeftLaneChange";
    case BehaviorState::kRightLaneChangePreparation:
      return out << "RightLaneChangePreparation";
    case BehaviorState::kRightLaneChange:
      return out << "RightLaneChange";
    default:
      throw std::runtime_error("Not supported BehaviorState.");
  }
}

} // namespace pathplanning
