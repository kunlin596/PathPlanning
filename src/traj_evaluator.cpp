#include "traj_evaluator.h"
#include "log.h"

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
TimeDiffCost::Compute(const JMTTrajectory2D& traj,
                      const VehicleConfiguration& goalConf,
                      const double requestTime,
                      const TrackedVehicleMap& trackedVehicleMap,
                      const JMTTrajectoryEvaluator::Options& options)
{
  // Flip gaussian distribution upside down
  static constexpr double SIGMA = 1.0; // seconds
  return GaussianLoss1D(requestTime, SIGMA, traj.GetTime());
}

double
SDiffCost::Compute(const JMTTrajectory2D& traj,
                   const VehicleConfiguration& goalConf,
                   const double requestTime,
                   const TrackedVehicleMap& trackedVehicleMap,
                   const JMTTrajectoryEvaluator::Options& options)
{
  double cost = 0.0;
  VehicleConfiguration finalConf = traj(traj.GetTime());
  cost += GaussianLoss1D(goalConf.sPos, options.evalSigmas[0], finalConf.sPos);
  cost += GaussianLoss1D(goalConf.sVel, options.evalSigmas[1], finalConf.sVel);
  cost += GaussianLoss1D(goalConf.sAcc, options.evalSigmas[2], finalConf.sAcc);
  return cost / 3.0;
}

double
DDiffCost::Compute(const JMTTrajectory2D& traj,
                   const VehicleConfiguration& goalConf,
                   const double requestTime,
                   const TrackedVehicleMap& trackedVehicleMap,
                   const JMTTrajectoryEvaluator::Options& options)
{
  double cost = 0.0;
  VehicleConfiguration finalConf = traj(traj.GetTime());
  cost += GaussianLoss1D(goalConf.dPos, options.evalSigmas[3], finalConf.dPos);
  cost += GaussianLoss1D(goalConf.dVel, options.evalSigmas[4], finalConf.dVel);
  cost += GaussianLoss1D(goalConf.dAcc, options.evalSigmas[5], finalConf.dAcc);
  return cost / 3.0;
}

double
CollisionCost::Compute(const JMTTrajectory2D& traj,
                       const VehicleConfiguration& goalConf,
                       const double requestTime,
                       const TrackedVehicleMap& trackedVehicleMap,
                       const JMTTrajectoryEvaluator::Options& options)
{
  double dist = traj.ComputeNearestApproach(
    trackedVehicleMap, traj.GetTime(), options.collisionCheckingTimeStep);
  double threshold = std::max(Vehicle::Size, options.collisionCheckingRadius);
  if (dist < threshold) {
    SPDLOG_INFO("Collision detected, closest dist={:7.3f}, threshold={:7.3f}",
                dist,
                threshold);
    return 1.0;
  }
  return 0.0;
}

double
BufferCost::Compute(const JMTTrajectory2D& traj,
                    const VehicleConfiguration& goalConf,
                    const double requestTime,
                    const TrackedVehicleMap& trackedVehicleMap,
                    const JMTTrajectoryEvaluator::Options& options)
{
  double dist = traj.ComputeNearestApproach(
    trackedVehicleMap, options.timeHorizon, options.timeStep);
  double threshold = std::max(Vehicle::Size, options.collisionCheckingRadius);
  static constexpr double SIGMA = Vehicle::Size; // meter
  return Gaussian1D(threshold, dist, SIGMA);
}

double
StaysOnRoadCost::Compute(const JMTTrajectory2D& traj,
                         const VehicleConfiguration& goalConf,
                         const double requestTime,
                         const TrackedVehicleMap& trackedVehicleMap,
                         const JMTTrajectoryEvaluator::Options& options)
{
  double minDist = std::numeric_limits<double>::infinity();
  double currTime = 0.0;
  double timeStep = options.timeStep;
  while (currTime < (traj.GetTime() + 1e-6)) {
    currTime += timeStep;
    VehicleConfiguration trajConf = traj.Eval(currTime);
    if (not Map::IsInRoad(trajConf.dPos)) {
      SPDLOG_DEBUG("{:7.3f}: {}", trajConf.dPos, Map::IsInRoad(trajConf.dPos));
      return 1.0;
    }
  }
  return 0.0;
}
double
ExceedsSpeedLimitCost::Compute(const JMTTrajectory2D& traj,
                               const VehicleConfiguration& goalConf,
                               const double requestTime,
                               const TrackedVehicleMap& trackedVehicleMap,
                               const JMTTrajectoryEvaluator::Options& options)
{
  // TODO
  return 0.0;
}
double
EfficiencyCost::Compute(const JMTTrajectory2D& traj,
                        const VehicleConfiguration& goalConf,
                        const double requestTime,
                        const TrackedVehicleMap& trackedVehicleMap,
                        const JMTTrajectoryEvaluator::Options& options)
{
  VehicleConfiguration finalConf = traj(traj.GetTime());
  double avgVel = finalConf.sPos / traj.GetTime();
  return Logistic(2.0 * std::abs(goalConf.sVel - avgVel) / avgVel);
}
double
TotalAccelCost::Compute(const JMTTrajectory2D& traj,
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
MaxAccelCost::Compute(const JMTTrajectory2D& traj,
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
TotalJerkCost::Compute(const JMTTrajectory2D& traj,
                       const VehicleConfiguration& goalConf,
                       const double requestTime,
                       const TrackedVehicleMap& trackedVehicleMap,
                       const JMTTrajectoryEvaluator::Options& options)
{
  // TODO: Check D as well
  double currTime = 0.0;
  double totalSJerk = 0.0;
  auto sJerkFunc = traj.GetSAccFunc().Differentiate();
  while (currTime < options.timeHorizon) {
    totalSJerk += std::abs(sJerkFunc(currTime) * options.timeStep);
    currTime += options.timeStep;
  }
  double sJerkPerSec = totalSJerk / options.timeHorizon;

  return Logistic(sJerkPerSec / options.expectedJerkInOneSec);
}

double
MaxJerkCost::Compute(const JMTTrajectory2D& traj,
                     const VehicleConfiguration& goalConf,
                     const double requestTime,
                     const TrackedVehicleMap& trackedVehicleMap,
                     const JMTTrajectoryEvaluator::Options& options)
{
  // TODO: Check D as well
  double currTime = 0.0;
  auto sJerkFunc = traj.GetSAccFunc().Differentiate();
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
JMTTrajectoryEvaluator::Evaluate(const JMTTrajectory2D& traj,
                                 const VehicleConfiguration& goalConf,
                                 const double requestTime,
                                 const TrackedVehicleMap& trackedVehicleMap)
{
  double totalCost = 0.0;
  for (const auto& costInfo : _options.driverProfile) {
    if (costInfo.second < 0.0) {
      continue;
    }
    if (_funcPtrs.count(costInfo.first) == 0) {
      _funcPtrs[costInfo.first] = costs::CreateCostFunctor(costInfo.first);
    }
    double cost = _funcPtrs[costInfo.first]->Compute(
                    traj, goalConf, requestTime, trackedVehicleMap, _options) *
                  costInfo.second;
    totalCost += cost;
    SPDLOG_DEBUG("    {:20s}: {:7.3}", costInfo.first, cost);
  }
  return totalCost;
}

namespace costs {

std::ostream&
operator<<(std::ostream& out, const CostType& type)
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

} // namespace costs
} // namespace pathplanning
