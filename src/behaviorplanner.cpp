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
// std::unordered_map<std::string, Vehicle>
// _GetFrontBackVehicles(const Matrix32d& egoConf,
//                       const TrackedVehicleMap& trackedVehicleMap,
//                       int targetLaneId = -1,
//                       double nonEgoSearchRadius = 30.0)
// {
//   double frontSDiff = std::numeric_limits<double>::max();
//   double backSDiff = std::numeric_limits<double>::max();

//   int frontId = -1;
//   int backId = -1;

//   for (const auto& vehicleData : trackedVehicleMap) {
//     const auto& vehicleConf = vehicleData.second.PredictConfiguration();

//     int carLaneId = Map::GetLaneId(vehicleConf.kinematics[3]);
//     if (carLaneId < 0) {
//       continue;
//     }

//     if (carLaneId == targetLaneId) {
//       double currFrontSDiff = vehicleConf.kinematics[0] - egoConf.kinematics[0];
//       double currBackSDiff = egoConf.kinematics[0] - vehicleConf.kinematics[0];

//       if (0.0 < currFrontSDiff and currFrontSDiff < frontSDiff) {
//         frontSDiff = currFrontSDiff;
//         frontId = vehicleData.first;

//       } else if (0.0 < currBackSDiff and currBackSDiff < backSDiff) {
//         backSDiff = currBackSDiff;
//         backId = vehicleData.first;
//       }
//     }
//   }
//   std::unordered_map<std::string, Vehicle> result;

//   if (frontId != -1 and frontSDiff < nonEgoSearchRadius) {
//     result["front"] = trackedVehicleMap.at(frontId);
//     SPDLOG_DEBUG("Found front id {:2d}, frontSDiff={:7.3f}", frontId, frontSDiff);
//   }
//   if (backId != -1 and backSDiff < nonEgoSearchRadius) {
//     result["back"] = trackedVehicleMap.at(backId);
//     SPDLOG_DEBUG("Found back id {:2d}, backSDiff={:7.3f}", backId, backSDiff);
//   }
//   return result;
// }

Matrix32d
_GenerateLKProposal(const Matrix32d& egoConf, const TrackedVehicleMap& trackedVehicleMap, const Configuration& conf)
{
  Matrix32d proposal;
  proposal.col(0) << egoConf(0, 0) + 30.0, Mph2Mps(49.5), 0.0;
  proposal.col(1) << Map::GetLaneCenterD(Map::GetLaneId(egoConf(0, 1))), 0.0, 0.0;
  return proposal;
}

Matrix32d
_GenerateLCPProposal(const Matrix32d& egoConf,
                     const TrackedVehicleMap& trackedVehicleMap,
                     const Configuration& conf,
                     int laneOffset = -1)
{
  return Matrix32d();
}

Matrix32d
_GenerateLLCPProposal(const Matrix32d& egoConf, const TrackedVehicleMap& trackedVehicleMap, const Configuration& conf)
{
  return _GenerateLCPProposal(egoConf, trackedVehicleMap, conf, -1);
}

Matrix32d
_GenerateRLCPProposal(const Matrix32d& egoConf, const TrackedVehicleMap& trackedVehicleMap, const Configuration& conf)
{
  return _GenerateLCPProposal(egoConf, trackedVehicleMap, conf, -1);
}

Matrix32d
_GenerateLCProposal(const Matrix32d& egoConf,
                    const TrackedVehicleMap& trackedVehicleMap,
                    const Configuration& conf,
                    int laneOffset = -1)
{
  return Matrix32d();
}

Matrix32d
_GenerateLLCProposal(const Matrix32d& egoConf, const TrackedVehicleMap& trackedVehicleMap, const Configuration& conf)
{
  return _GenerateLCProposal(egoConf, trackedVehicleMap, conf, -1);
}

Matrix32d
_GenerateRLCProposal(const Matrix32d& egoConf, const TrackedVehicleMap& trackedVehicleMap, const Configuration& conf)
{
  return _GenerateLCProposal(egoConf, trackedVehicleMap, conf, -1);
}

std::vector<BehaviorState>
_GetSuccessorStates(const BehaviorState& state)
{
  std::vector<BehaviorState> states = { BehaviorState::kLaneKeeping };
  // switch (state) {
  //   case BehaviorState::kReady:
  //     break;
  //   case BehaviorState::kLaneKeeping:
  //     states.push_back(BehaviorState::kLeftLaneChangePreparation);
  //     states.push_back(BehaviorState::kRightLaneChangePreparation);
  //     break;
  //   case BehaviorState::kLeftLaneChangePreparation:
  //     states.push_back(BehaviorState::kLeftLaneChangePreparation);
  //     states.push_back(BehaviorState::kLeftLaneChange);
  //     break;
  //   case BehaviorState::kRightLaneChangePreparation:
  //     states.push_back(BehaviorState::kRightLaneChangePreparation);
  //     states.push_back(BehaviorState::kRightLaneChange);
  //     break;
  //   case BehaviorState::kLeftLaneChange:
  //     states.push_back(BehaviorState::kLeftLaneChange);
  //     break;
  //   case BehaviorState::kRightLaneChange:
  //     states.push_back(BehaviorState::kRightLaneChange);
  //     break;
  //   default:
  //     break;
  // }
  return states;
}

} // namespace

namespace pathplanning {

BehaviorPlanner::BehaviorPlanner(const Map& map, const Configuration& conf)
  : _map(map)
  , _conf(conf)
{
  // _pEvaluator =
  //   std::make_unique<JMTTrajectoryEvaluator>(_conf.trajEvaluationOptions);
}

JMTTrajectory2d
BehaviorPlanner::GenerateProposal(const Vehicle& ego, const TrackedVehicleMap& trackedVehicleMap)
{
  Matrix32d egoKinematics = ego.GetKinematics(0.0);

  double minCost = std::numeric_limits<double>::max();
  BehaviorState bestState;
  JMTTrajectory2d bestTraj;

  SPDLOG_DEBUG("egoKinematics={}", egoKinematics);
  for (const auto& state : _GetSuccessorStates(_currState)) {
    Matrix32d proposalKinematics;

    switch (state) {
      case BehaviorState::kLaneKeeping:
        proposalKinematics = _GenerateLKProposal(egoKinematics, trackedVehicleMap, _conf);
        break;
      case BehaviorState::kLeftLaneChangePreparation:
        proposalKinematics = _GenerateLLCPProposal(egoKinematics, trackedVehicleMap, _conf);
        break;
      case BehaviorState::kLeftLaneChange:
        proposalKinematics = _GenerateLLCProposal(egoKinematics, trackedVehicleMap, _conf);
        break;
      case BehaviorState::kRightLaneChangePreparation:
        proposalKinematics = _GenerateRLCPProposal(egoKinematics, trackedVehicleMap, _conf);
        break;
      case BehaviorState::kRightLaneChange:
        proposalKinematics = _GenerateRLCProposal(egoKinematics, trackedVehicleMap, _conf);
        break;
      default:
        break;
    }

    SPDLOG_DEBUG("proposalKinematics={}", proposalKinematics);
    Matrix62d conditions;
    conditions.block<3, 2>(0, 0) = egoKinematics;
    conditions.block<3, 2>(3, 0) = proposalKinematics;
    std::vector<JMTTrajectory2d> feasibleTrajectories = JMT::SolveMultipleFeasible2d(conditions, _map, _conf);

    for (auto& traj : feasibleTrajectories) {
      SPDLOG_DEBUG(traj);
    }
    if (!feasibleTrajectories.empty()) {
      bestTraj = feasibleTrajectories[feasibleTrajectories.size() - 1];
    }

    // for (const auto& traj : feasibleTrajectories) {

    //   double cost = _pEvaluator->Evaluate(traj, proposalState, _conf.timeHorizon, trackedVehicleMap);

    //   SPDLOG_DEBUG("  Current cost {:s}: {:7.3}", state, cost);
    //   if (cost < minCost) {
    //     minCost = cost;
    //     bestState = state;
    //     bestTraj = traj;
    //     SPDLOG_INFO("  Update best {:s}: {:7.3}", bestState, minCost);
    //   }
    // }
  }

  _currState = bestState;
  return bestTraj;
}

std::ostream&
operator<<(std::ostream& out, const BehaviorState& type)
{
  using namespace pathplanning;
  switch (type) {
    case BehaviorState::kReady:
      return out << "Ready";
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
