#include "traj_evaluator.h"

namespace pathplanning {
namespace costs {
std::shared_ptr<CostFunctorBase> CreateCostFunctor(const CostType& type) {
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

double TimeDiffCost::Compute(const JMTTrajectory& traj,
                             const VehicleConfiguration& goalConf,
                             const double requestTime,
                             const Predictions& predictions) {
  return Logistic(
      static_cast<double>(std::abs(traj.elapsedTime - requestTime)) /
      requestTime);
}

double SDiffCost::Compute(const JMTTrajectory& traj,
                          const VehicleConfiguration& goalConf,
                          const double requestTime,
                          const Predictions& predictions) {
  return 0.0;
}

double DDiffCost::Compute(const JMTTrajectory& traj,
                          const VehicleConfiguration& goalConf,
                          const double requestTime,
                          const Predictions& predictions) {
  return 0.0;
}

double CollisionCost::Compute(const JMTTrajectory& traj,
                              const VehicleConfiguration& goalConf,
                              const double requestTime,
                              const Predictions& predictions) {
  return 0.0;
}
double BufferCost::Compute(const JMTTrajectory& traj,
                           const VehicleConfiguration& goalConf,
                           const double requestTime,
                           const Predictions& predictions) {
  return 0.0;
}
double StaysOnRoadCost::Compute(const JMTTrajectory& traj,
                                const VehicleConfiguration& goalConf,
                                const double requestTime,
                                const Predictions& predictions) {
  return 0.0;
}
double ExceedsSpeedLimitCost::Compute(const JMTTrajectory& traj,
                                      const VehicleConfiguration& goalConf,
                                      const double requestTime,
                                      const Predictions& predictions) {
  return 0.0;
}
double EfficiencyCost::Compute(const JMTTrajectory& traj,
                               const VehicleConfiguration& goalConf,
                               const double requestTime,
                               const Predictions& predictions) {
  return 0.0;
}
double TotalAccelCost::Compute(const JMTTrajectory& traj,
                               const VehicleConfiguration& goalConf,
                               const double requestTime,
                               const Predictions& predictions) {
  return 0.0;
}
double MaxAccelCost::Compute(const JMTTrajectory& traj,
                             const VehicleConfiguration& goalConf,
                             const double requestTime,
                             const Predictions& predictions) {
  return 0.0;
}
double TotalJerkCost::Compute(const JMTTrajectory& traj,
                              const VehicleConfiguration& goalConf,
                              const double requestTime,
                              const Predictions& predictions) {
  return 0.0;
}
double MaxJerkCost::Compute(const JMTTrajectory& traj,
                            const VehicleConfiguration& goalConf,
                            const double requestTime,
                            const Predictions& predictions) {
  return 0.0;
}
}  // namespace costs

double JMTTrajectoryEvaluator::Evaluate(const JMTTrajectory& traj,
                                        const VehicleConfiguration& goalConf,
                                        const double requestTime,
                                        const Predictions& predictions) {
  double cost = 0.0;
  for (const auto& costInfo : _costWeightMapping) {
    if (_funcPtrs.count(costInfo.first) == 0) {
      _funcPtrs[costInfo.first] = costs::CreateCostFunctor(costInfo.first);
    }
    cost += _funcPtrs[costInfo.first]->Compute(traj, goalConf, requestTime,
                                               predictions) *
            costInfo.second;
  }
  return cost;
}

}  // namespace pathplanning
