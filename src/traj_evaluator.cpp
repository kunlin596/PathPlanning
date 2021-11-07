#include "traj_evaluator.h"
#include "collision_checker.h"
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
    case CostType::kBuffer:
      return std::make_shared<BufferCost>();
    case CostType::kEfficiency:
      return std::make_shared<EfficiencyCost>();
    case CostType::kTotalAccel:
      return std::make_shared<TotalAccelCost>();
    case CostType::kTotalJerk:
      return std::make_shared<TotalJerkCost>();
    default:
      throw std::runtime_error("Not supported cost function");
  }
}

double
TimeDiffCost::operator()(const JMTTrajectory2d& traj,
                         const Matrix32d& goalConf,
                         const double requestTime,
                         const TrackedVehicleMap& trackedVehicleMap,
                         const Configuration& conf)
{
  // Flip gaussian distribution upside down
  static constexpr double SIGMA = 1.0; // seconds
  return GaussianLoss1D(requestTime, SIGMA, traj.GetTime());
}

double
SDiffCost::operator()(const JMTTrajectory2d& traj,
                      const Matrix32d& goalConf,
                      const double requestTime,
                      const TrackedVehicleMap& trackedVehicleMap,
                      const Configuration& conf)
{
  double cost = 0.0;
  Matrix62d finalConf = traj(traj.GetTime());
  cost += GaussianLoss1D(goalConf(0, 0), conf.trajectory.evalSigmas[0], finalConf(0, 0));
  cost += GaussianLoss1D(goalConf(0, 1), conf.trajectory.evalSigmas[1], finalConf(0, 1));
  cost += GaussianLoss1D(goalConf(0, 2), conf.trajectory.evalSigmas[2], finalConf(0, 2));
  return cost / 3.0;
}

double
DDiffCost::operator()(const JMTTrajectory2d& traj,
                      const Matrix32d& goalConf,
                      const double requestTime,
                      const TrackedVehicleMap& trackedVehicleMap,
                      const Configuration& conf)
{
  double cost = 0.0;
  Matrix62d finalConf = traj(traj.GetTime());
  cost += GaussianLoss1D(goalConf(1, 0), conf.trajectory.evalSigmas[3], finalConf(1, 0));
  cost += GaussianLoss1D(goalConf(1, 1), conf.trajectory.evalSigmas[4], finalConf(1, 1));
  cost += GaussianLoss1D(goalConf(1, 2), conf.trajectory.evalSigmas[5], finalConf(1, 2));
  return cost / 3.0;
}

double
BufferCost::operator()(const JMTTrajectory2d& traj,
                       const Matrix32d& goalConf,
                       const double requestTime,
                       const TrackedVehicleMap& trackedVehicleMap,
                       const Configuration& conf)
{
  const auto& [id, dist] = CollisionChecker::GetMinDistance(traj, trackedVehicleMap, conf.timeHorizon, conf.timeStep);
  double threshold = std::max(Vehicle::Size, conf.trajectory.collisionCheckingRadius);
  static constexpr double SIGMA = Vehicle::Size; // meter
  return Gaussian1D(threshold, dist, SIGMA);
}

double
EfficiencyCost::operator()(const JMTTrajectory2d& traj,
                           const Matrix32d& goalConf,
                           const double requestTime,
                           const TrackedVehicleMap& trackedVehicleMap,
                           const Configuration& conf)
{
  Matrix62d finalConf = traj(traj.GetTime());
  double avgVel = finalConf(0, 1) / traj.GetTime();
  static constexpr double SIGMA = 40.0; // meters / second
  // NOTE: Should not reward lateral d velocity
  return GaussianLoss1D(goalConf(0, 1), SIGMA, avgVel);
}

double
TotalAccelCost::operator()(const JMTTrajectory2d& traj,
                           const Matrix32d& goalConf,
                           const double requestTime,
                           const TrackedVehicleMap& trackedVehicleMap,
                           const Configuration& conf)
{
  // TODO: Check D as well
  double currTime = 0.0;
  double totalSAcc = 0.0;
  double totalDAcc = 0.0;

  while (currTime < conf.timeHorizon) {
    Matrix62d currConf = traj(currTime);
    totalSAcc += std::abs(currConf(0, 2) * conf.timeStep);
    totalDAcc += std::abs(currConf(1, 2) * conf.timeStep);
    currTime += conf.timeStep;
  }
  double sAccPerSec = totalSAcc / conf.timeHorizon;
  double dAccPerSec = totalDAcc / conf.timeHorizon;

  return (Logistic(sAccPerSec / conf.trajectory.expectedAccInOneSec) +
          Logistic(dAccPerSec / conf.trajectory.expectedAccInOneSec)) /
         2.0;
}

double
TotalJerkCost::operator()(const JMTTrajectory2d& traj,
                          const Matrix32d& goalConf,
                          const double requestTime,
                          const TrackedVehicleMap& trackedVehicleMap,
                          const Configuration& conf)
{
  double currTime = 0.0;
  double totalSJerk = 0.0;
  double totalDJerk = 0.0;

  auto sJerkFunc = traj.GetTraj1().GetFunc2();
  auto dJerkFunc = traj.GetTraj2().GetFunc2();

  while (currTime < conf.timeHorizon) {
    totalSJerk += std::abs(sJerkFunc(currTime) * conf.timeStep);
    totalDJerk += std::abs(dJerkFunc(currTime) * conf.timeStep);
    currTime += conf.timeStep;
  }

  double sJerkPerSec = totalSJerk / conf.timeHorizon;
  double dJerkPerSec = totalDJerk / conf.timeHorizon;

  return (Logistic(sJerkPerSec / conf.trajectory.expectedJerkInOneSec) +
          Logistic(dJerkPerSec / conf.trajectory.expectedJerkInOneSec)) /
         2.0;
}

} // namespace costs

double
JMTTrajectoryEvaluator::Evaluate(const JMTTrajectory2d& traj,
                                 const Matrix32d& goalConf,
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

    double cost = (*_funcPtrs[costInfo.first])(traj, goalConf, requestTime, trackedVehicleMap, _conf) * costInfo.second;
    totalCost += cost;
    SPDLOG_TRACE("        - {:20s}: weight={}, cost={:10.3f}", costInfo.first, costInfo.second, cost);
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
    case CostType::kBuffer:
      return out << "BufferCost";
    case CostType::kEfficiency:
      return out << "EfficiencyCost";
    case CostType::kTotalAccel:
      return out << "TotalAccelCost";
    case CostType::kTotalJerk:
      return out << "TotalJerkCost";
    default:
      throw std::runtime_error("Not supported cost function");
  }
}

} // namespace costs

JMTTrajectoryEvaluator::Options::Options(const Configuration& conf)
{
  using namespace ::pathplanning::costs;
  driverProfileName = conf.driverProfileName;
  driverProfile[CostType::kTimeDiff] = conf.driverProfile.timeDiff;
  driverProfile[CostType::kSDiff] = conf.driverProfile.sDiff;
  driverProfile[CostType::kDDiff] = conf.driverProfile.dDiff;
  driverProfile[CostType::kBuffer] = conf.driverProfile.buffer;
  driverProfile[CostType::kEfficiency] = conf.driverProfile.efficiency;
  driverProfile[CostType::kTotalAccel] = conf.driverProfile.totalAcc;
  driverProfile[CostType::kTotalJerk] = conf.driverProfile.totalJerk;
}

} // namespace pathplanning
