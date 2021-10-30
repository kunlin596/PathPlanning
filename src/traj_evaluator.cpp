#include "traj_evaluator.h"

namespace pathplanning {
namespace costs {
std::shared_ptr<CostFunctorBase>
CreateCostFunctor(const CostType& type)
{
  switch (type) {
    case CostType::kTimeDiff:
      return std::make_shared<TimeDiffCost>();
    case CostType::kSDiff:
      return std::make_shared<SDiffCost>();
    case CostType::kDDiff:
      return std::make_shared<DDiffCost>();
    case CostType::kCollision:
      return std::make_shared<CollisionCost>();
    case CostType::kBuffer:
      return std::make_shared<BufferCost>();
    case CostType::kStaysOnRoad:
      return std::make_shared<StaysOnRoadCost>();
    case CostType::kExceedsSpeedLimit:
      return std::make_shared<ExceedsSpeedLimitCost>();
    case CostType::kEfficiency:
      return std::make_shared<EfficiencyCost>();
    case CostType::kTotalAccel:
      return std::make_shared<TotalAccelCost>();
    case CostType::kMaxAccel:
      return std::make_shared<MaxAccelCost>();
    case CostType::kTotalJerk:
      return std::make_shared<TotalJerkCost>();
    case CostType::kMaxJerk:
      return std::make_shared<MaxJerkCost>();
    default:
      throw std::runtime_error("Not supported cost function");
  }
}

double
TimeDiffCost::Compute(const JMTTrajectory& traj,
                      const VehicleConfiguration& goalConf,
                      const double requestTime,
                      const TrackedVehicleMap& trackedVehicleMap,
                      const JMTTrajectoryEvaluator::Options& options)
{
  return Logistic(
    static_cast<double>(std::abs(traj.elapsedTime - requestTime)) /
    requestTime);
}

double
SDiffCost::Compute(const JMTTrajectory& traj,
                   const VehicleConfiguration& goalConf,
                   const double requestTime,
                   const TrackedVehicleMap& trackedVehicleMap,
                   const JMTTrajectoryEvaluator::Options& options)
{
  double cost = 0.0;
  VehicleConfiguration finalConf = traj(traj.elapsedTime);
  cost +=
    Logistic(std::abs(finalConf.sPos - goalConf.sPos) / options.evalSigmas[0]);
  cost +=
    Logistic(std::abs(finalConf.sVel - goalConf.sVel) / options.evalSigmas[1]);
  cost +=
    Logistic(std::abs(finalConf.sAcc - goalConf.sAcc) / options.evalSigmas[2]);

  return cost;
}

double
DDiffCost::Compute(const JMTTrajectory& traj,
                   const VehicleConfiguration& goalConf,
                   const double requestTime,
                   const TrackedVehicleMap& trackedVehicleMap,
                   const JMTTrajectoryEvaluator::Options& options)
{
  double cost = 0.0;
  VehicleConfiguration finalConf = traj(traj.elapsedTime);
  cost +=
    Logistic(std::abs(finalConf.dPos - goalConf.dPos) / options.evalSigmas[3]);
  cost +=
    Logistic(std::abs(finalConf.dVel - goalConf.dVel) / options.evalSigmas[4]);
  cost +=
    Logistic(std::abs(finalConf.dAcc - goalConf.dAcc) / options.evalSigmas[5]);
  return cost;
}

double
CollisionCost::Compute(const JMTTrajectory& traj,
                       const VehicleConfiguration& goalConf,
                       const double requestTime,
                       const TrackedVehicleMap& trackedVehicleMap,
                       const JMTTrajectoryEvaluator::Options& options)
{
  double dist = traj.ComputeNearestApproach(
    trackedVehicleMap, options.timeHorizon, options.timeStep);
  if (dist < options.collisionCheckingRadius) {
    return 1.0;
  }
  return 0.0;
}
double
BufferCost::Compute(const JMTTrajectory& traj,
                    const VehicleConfiguration& goalConf,
                    const double requestTime,
                    const TrackedVehicleMap& trackedVehicleMap,
                    const JMTTrajectoryEvaluator::Options& options)
{
  double dist = traj.ComputeNearestApproach(
    trackedVehicleMap, options.timeHorizon, options.timeStep);
  return Logistic(options.collisionCheckingRadius / dist);
}
double
StaysOnRoadCost::Compute(const JMTTrajectory& traj,
                         const VehicleConfiguration& goalConf,
                         const double requestTime,
                         const TrackedVehicleMap& trackedVehicleMap,
                         const JMTTrajectoryEvaluator::Options& options)
{
  // TODO
  return 0.0;
}
double
ExceedsSpeedLimitCost::Compute(const JMTTrajectory& traj,
                               const VehicleConfiguration& goalConf,
                               const double requestTime,
                               const TrackedVehicleMap& trackedVehicleMap,
                               const JMTTrajectoryEvaluator::Options& options)
{
  // TODO
  return 0.0;
}
double
EfficiencyCost::Compute(const JMTTrajectory& traj,
                        const VehicleConfiguration& goalConf,
                        const double requestTime,
                        const TrackedVehicleMap& trackedVehicleMap,
                        const JMTTrajectoryEvaluator::Options& options)
{
  VehicleConfiguration finalConf = traj(traj.elapsedTime);
  double avgVel = finalConf.sPos / traj.elapsedTime;
  return Logistic(2.0 * std::abs(goalConf.sVel - avgVel) / avgVel);
}
double
TotalAccelCost::Compute(const JMTTrajectory& traj,
                        const VehicleConfiguration& goalConf,
                        const double requestTime,
                        const TrackedVehicleMap& trackedVehicleMap,
                        const JMTTrajectoryEvaluator::Options& options)
{
  // TODO: Check D as well
  double currTime = 0.0;
  double totalSAcc = 0.0;
  while (currTime < options.timeHorizon) {
    VehicleConfiguration currConf = traj(currTime);
    totalSAcc += std::abs(currConf.sAcc * options.timeStep);
    currTime += options.timeStep;
  }
  double sAccPerSec = totalSAcc / options.timeHorizon;

  return Logistic(sAccPerSec / options.expectedAccInOneSec);
}
double
MaxAccelCost::Compute(const JMTTrajectory& traj,
                      const VehicleConfiguration& goalConf,
                      const double requestTime,
                      const TrackedVehicleMap& trackedVehicleMap,
                      const JMTTrajectoryEvaluator::Options& options)
{
  // TODO: Check D as well
  double currTime = 0.0;
  while (currTime < options.timeHorizon) {
    VehicleConfiguration currConf = traj(currTime);
    if (currConf.sAcc > options.maxAcc) {
      return 1.0;
    }
    currTime += options.timeStep;
  }
  return 0.0;
}

double
TotalJerkCost::Compute(const JMTTrajectory& traj,
                       const VehicleConfiguration& goalConf,
                       const double requestTime,
                       const TrackedVehicleMap& trackedVehicleMap,
                       const JMTTrajectoryEvaluator::Options& options)
{
  // TODO: Check D as well
  double currTime = 0.0;
  double totalSJerk = 0.0;
  auto sJerkFunc = traj.GetSDFunc().GetSAccFunc().Differentiate();
  while (currTime < options.timeHorizon) {
    totalSJerk += std::abs(sJerkFunc(currTime) * options.timeStep);
    currTime += options.timeStep;
  }
  double sJerkPerSec = totalSJerk / options.timeHorizon;

  return Logistic(sJerkPerSec / options.expectedJerkInOneSec);
}

double
MaxJerkCost::Compute(const JMTTrajectory& traj,
                     const VehicleConfiguration& goalConf,
                     const double requestTime,
                     const TrackedVehicleMap& trackedVehicleMap,
                     const JMTTrajectoryEvaluator::Options& options)
{
  // TODO: Check D as well
  double currTime = 0.0;
  auto sJerkFunc = traj.GetSDFunc().GetSAccFunc().Differentiate();
  while (currTime < options.timeHorizon) {
    double jerk = sJerkFunc(currTime);
    if (jerk > options.maxJerk) {
      return 1.0;
    }
    currTime += options.timeStep;
  }
  return 0.0;
}
} // namespace costs

double
JMTTrajectoryEvaluator::Evaluate(const JMTTrajectory& traj,
                                 const VehicleConfiguration& goalConf,
                                 const double requestTime,
                                 const TrackedVehicleMap& trackedVehicleMap)
{
  double totalCost = 0.0;
  for (const auto& costInfo : _costWeightMapping) {
    if (_funcPtrs.count(costInfo.first) == 0) {
      _funcPtrs[costInfo.first] = costs::CreateCostFunctor(costInfo.first);
    }
    double cost = _funcPtrs[costInfo.first]->Compute(
                    traj, goalConf, requestTime, trackedVehicleMap, _options) *
                  costInfo.second;
    totalCost += cost;
    SPDLOG_ERROR("    {:20s}: {:7.3}", costInfo.first, cost);
  }
  return totalCost;
}

inline std::ostream&
operator<<(std::ostream& out, const costs::CostType& type)
{
  using namespace pathplanning::costs;
  switch (type) {
    case CostType::kTimeDiff:
      return out << "TimeDiffCost";
    case CostType::kSDiff:
      return out << "SDiffCost";
    case CostType::kDDiff:
      return out << "DDiffCost";
    case CostType::kCollision:
      return out << "CollisionCost";
    case CostType::kBuffer:
      return out << "BufferCost";
    case CostType::kStaysOnRoad:
      return out << "StaysOnRoadCost";
    case CostType::kExceedsSpeedLimit:
      return out << "ExceedsSpeedLimitCost";
    case CostType::kEfficiency:
      return out << "EfficiencyCost";
    case CostType::kTotalAccel:
      return out << "TotalAccelCost";
    case CostType::kMaxAccel:
      return out << "MaxAccelCost";
    case CostType::kTotalJerk:
      return out << "TotalJerkCost";
    case CostType::kMaxJerk:
      return out << "MaxJerkCost";
    default:
      throw std::runtime_error("Not supported cost function");
  }
}

} // namespace pathplanning
