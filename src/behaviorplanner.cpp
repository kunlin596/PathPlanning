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
  proposal.col(0) << egoConf(0, 0) + conf.trajectory.planningDistance, 0.0, 0.0;
  proposal.col(1) << Map::GetLaneCenterD(Map::GetLaneId(egoConf(0, 1))), 0.0, 0.0;
  return proposal;
}

Matrix32d
_GenerateLCPProposal(const Matrix32d& egoConf,
                     const TrackedVehicleMap& trackedVehicleMap,
                     const Configuration& conf,
                     int laneOffset = -1)
{
  // TODO
  Matrix32d proposal;
  proposal.col(0) << egoConf(0, 0) + conf.trajectory.planningDistance, 0.0, 0.0;
  proposal.col(1) << Map::GetLaneCenterD(Map::GetLaneId(egoConf(0, 1))), 0.0, 0.0;
  return proposal;
}

Matrix32d
_GenerateLLCPProposal(const Matrix32d& egoConf, const TrackedVehicleMap& trackedVehicleMap, const Configuration& conf)
{
  return _GenerateLCPProposal(egoConf, trackedVehicleMap, conf, -1);
}

Matrix32d
_GenerateRLCPProposal(const Matrix32d& egoConf, const TrackedVehicleMap& trackedVehicleMap, const Configuration& conf)
{
  return _GenerateLCPProposal(egoConf, trackedVehicleMap, conf, +1);
}

Matrix32d
_GenerateLCProposal(const Matrix32d& egoConf,
                    const TrackedVehicleMap& trackedVehicleMap,
                    const Configuration& conf,
                    int laneOffset = -1)
{
  Matrix32d proposal;
  proposal.col(0) << egoConf(0, 0) + conf.trajectory.planningDistance, 0.0, 0.0;
  proposal.col(1) << Map::GetLaneCenterD(Map::GetLaneId(egoConf(0, 1)) + laneOffset), 0.0, 0.0;
  return proposal;
}

Matrix32d
_GenerateLLCProposal(const Matrix32d& egoConf, const TrackedVehicleMap& trackedVehicleMap, const Configuration& conf)
{
  return _GenerateLCProposal(egoConf, trackedVehicleMap, conf, -1);
}

Matrix32d
_GenerateRLCProposal(const Matrix32d& egoConf, const TrackedVehicleMap& trackedVehicleMap, const Configuration& conf)
{
  return _GenerateLCProposal(egoConf, trackedVehicleMap, conf, +1);
}

struct LaneStats
{
  double speed = 0.0;
  int numVehicles = 0.0;
};

std::unordered_map<int, LaneStats>
_ComputeLaneStats(double egoS, const TrackedVehicleMap& trackedVehicleMap)
{
  std::unordered_map<int, LaneStats> statsMap;
  for (const auto& [id, vehicle] : trackedVehicleMap) {
    int laneId = Map::GetLaneId(vehicle.GetKinematics(0.0)(0, 1));
    if (laneId < 0) {
      continue;
    }
    if (statsMap.count(laneId) == 0) {
      statsMap[laneId] = LaneStats();
    }
    statsMap[laneId].numVehicles++;
    statsMap[laneId].speed += vehicle.GetKinematics(0.0)(1, 0); // only count the s speed
  }

  for (auto& [laneId, landStats] : statsMap) {
    landStats.speed /= static_cast<double>(landStats.numVehicles);
    SPDLOG_INFO("lane id={:2d}, speed={:7.3f}, count={:2d}", laneId, landStats.speed, landStats.numVehicles);
  }
  return statsMap;
}

std::vector<BehaviorState>
_GetSuccessorStates(const Matrix32d& startKinematics,
                    const BehaviorState& state,
                    const TrackedVehicleMap& trackedVehicleMap)
{
  std::vector<BehaviorState> states;
  double s = startKinematics(0, 0);
  double d = startKinematics(0, 1);
  int laneId = Map::GetLaneId(d);

  std::unordered_map<int, LaneStats> laneStatsMap = _ComputeLaneStats(s, trackedVehicleMap);

  static int targetLane = laneId;
  int leftLaneId = laneId - 1;
  int rightLaneId = laneId + 1;

  switch (state) {
    case BehaviorState::kReady:
      states.push_back(BehaviorState::kLaneKeeping);
      break;
    case BehaviorState::kLaneKeeping:
      if (laneStatsMap.count(leftLaneId) and laneStatsMap[leftLaneId].speed > laneStatsMap[laneId].speed) {
        targetLane = leftLaneId;
        states.push_back(BehaviorState::kLeftLaneChangePreparation);
      } else if (laneStatsMap.count(rightLaneId) and laneStatsMap[rightLaneId].speed > laneStatsMap[laneId].speed) {
        targetLane = rightLaneId;
        states.push_back(BehaviorState::kRightLaneChangePreparation);
      } else {
        targetLane = laneId;
        states.push_back(BehaviorState::kLaneKeeping);
      }
      break;
    case BehaviorState::kLeftLaneChangePreparation:
      if (laneStatsMap.count(leftLaneId) and laneStatsMap[leftLaneId].numVehicles < 2) {
        states.push_back(BehaviorState::kLeftLaneChange);
      } else {
        states.push_back(BehaviorState::kLeftLaneChangePreparation);
      }
      break;
    case BehaviorState::kRightLaneChangePreparation:
      if (laneStatsMap.count(rightLaneId) and laneStatsMap[rightLaneId].numVehicles < 2) {
        states.push_back(BehaviorState::kRightLaneChange);
      } else {
        states.push_back(BehaviorState::kRightLaneChangePreparation);
      }
      break;
    case BehaviorState::kLeftLaneChange:
      if (targetLane != laneId or std::abs(d - Map::GetLaneCenterD(targetLane)) > 0.1) {
        states.push_back(BehaviorState::kLeftLaneChange);
      } else {
        states.push_back(BehaviorState::kLaneKeeping);
      }
      break;
    case BehaviorState::kRightLaneChange:
      if (targetLane != laneId or std::abs(d - Map::GetLaneCenterD(targetLane)) > 0.1) {
        states.push_back(BehaviorState::kRightLaneChange);
      } else {
        states.push_back(BehaviorState::kLaneKeeping);
      }
      break;
    default:
      states.push_back(BehaviorState::kLaneKeeping);
      break;
  }
  SPDLOG_INFO("targetLane={:2d}", targetLane);
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
BehaviorPlanner::GenerateProposal(const Matrix32d& startKinematics, const TrackedVehicleMap& trackedVehicleMap)
{
  double minCost = std::numeric_limits<double>::max();
  BehaviorState bestState;
  JMTTrajectory2d bestTraj;

  SPDLOG_DEBUG("startKinematics={}, {}", startKinematics.col(0).transpose(), startKinematics.col(1).transpose());

  std::unordered_map<BehaviorState, Matrix32d> proposalKinematicsMap;

  for (const auto& state : _GetSuccessorStates(startKinematics, _currState, trackedVehicleMap)) {
    switch (state) {
      case BehaviorState::kLaneKeeping:
        proposalKinematicsMap[BehaviorState::kLaneKeeping] =
          _GenerateLKProposal(startKinematics, trackedVehicleMap, _conf);
        break;
      case BehaviorState::kLeftLaneChangePreparation:
        proposalKinematicsMap[BehaviorState::kLeftLaneChangePreparation] =
          _GenerateLLCPProposal(startKinematics, trackedVehicleMap, _conf);
        break;
      case BehaviorState::kLeftLaneChange:
        proposalKinematicsMap[BehaviorState::kLeftLaneChange] =
          _GenerateLLCProposal(startKinematics, trackedVehicleMap, _conf);
        break;
      case BehaviorState::kRightLaneChangePreparation:
        proposalKinematicsMap[BehaviorState::kRightLaneChangePreparation] =
          _GenerateRLCPProposal(startKinematics, trackedVehicleMap, _conf);
        break;
      case BehaviorState::kRightLaneChange:
        proposalKinematicsMap[BehaviorState::kRightLaneChange] =
          _GenerateRLCProposal(startKinematics, trackedVehicleMap, _conf);
        break;
      default:
        break;
    }
  }

  for (const auto& [state, proposalKinematics] : proposalKinematicsMap) {
    Matrix62d conditions;
    conditions.block<3, 2>(0, 0) = startKinematics;
    conditions.block<3, 2>(3, 0) = proposalKinematics;
    std::vector<JMTTrajectory2d> feasibleTrajectories = JMT::SolveMultipleFeasible2d(conditions, _map, _conf);

    SPDLOG_INFO("state={:30s}, proposalKinematics={}, {}",
                state,
                proposalKinematics.col(0).transpose(),
                proposalKinematics.col(1).transpose());

    if (!feasibleTrajectories.empty()) {
      bestTraj = feasibleTrajectories[feasibleTrajectories.size() / 2];
      bestState = state;
    }
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
