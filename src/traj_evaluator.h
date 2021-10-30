#pragma once

#include <memory>
#include <unordered_map>

#include "jmt.h"

namespace pathplanning {

namespace costs {
enum class CostType
{
  kTimeDiff,
  kSDiff,
  kDDiff,
  kCollision,
  kBuffer,
  kStaysOnRoad,
  kExceedsSpeedLimit,
  kEfficiency,
  kTotalAccel,
  kMaxAccel,
  kTotalJerk,
  kMaxJerk
};

using CostWeightMapping = std::unordered_map<CostType, double>;

struct CostFunctorBase;

}

/**
 * @brief      This class describes a jmt trajectory evaluator.
 *
 * Given cost weight mapping, create the cost functions defined in the mapping
 * and sum up all of the cost w.r.t. give weight.
 */
class JMTTrajectoryEvaluator
{
public:
  struct Options
  {
    double timeHorizon = 2.0;
    double timeStep = 0.02;
    double collisionCheckingRadius = 30.0;
    double expectedAccInOneSec = 2.0;
    double expectedJerkInOneSec = 1.0;
    double maxAcc = 10.0;
    double maxJerk = 10.0;
    std::array<double, 6> evalSigmas = { 10.0, 1.0, 2.0, 1.0, 1.0, 1.0 };

    Options() {}
    Options(const Configuration& conf)
    {
      evalSigmas = conf.trajectoryEvaluation.evalSigmas;
      expectedAccInOneSec = conf.trajectoryEvaluation.expectedAccInOneSec;
      expectedJerkInOneSec = conf.trajectoryEvaluation.expectedJerkInOneSec;
      collisionCheckingRadius =
        conf.trajectoryEvaluation.collisionCheckingRadius;
      timeHorizon = conf.timeHorizon;
      timeStep = conf.timeStep;
    }
  };

  JMTTrajectoryEvaluator(const Options& options,
                         const costs::CostWeightMapping& costWeightMapping =
                           costs::CostWeightMapping())
    : _options(options)
    , _costWeightMapping(costWeightMapping)
  {}

  /**
   * @brief      Evaluate the trajecory
   *
   * @param[in]  traj         The traj
   * @param[in]  goalConf     The goal conf
   * @param[in]  requestTime  The request time
   * @param[in]  predictions  The predictions
   *
   * @return     Total cost
   */
  double Evaluate(const JMTTrajectory& traj,
                  const VehicleConfiguration& goalConf,
                  const double requestTime,
                  const TrackedVehicleMap& trackedVehicleMap);

private:
  costs::CostWeightMapping _costWeightMapping; ///< Cost function weight map
  std::unordered_map<costs::CostType, std::shared_ptr<costs::CostFunctorBase>>
    _funcPtrs; ///< Cost funtions
  Options _options;
};

namespace costs {

/**
 * @brief Cost functor base
 */
struct CostFunctorBase
{
  /**
   * @brief      Compute the cost for this trajectory
   *
   * `requestTime` is the original requested time duration for completing the
   * trajectory. Note that it's different from the time duration stored in the
   * trajectory. That is the actual planned time. Without time sampling the 2
   * values should be the same.
   *
   * @param[in]  traj         The trajectory
   * @param[in]  goalConf     The goal configuration
   * @param[in]  requestTime  The request time for the trajecory
   * @param[in]  predictions  The predictions
   *
   * @return     Cost value
   */
  virtual double Compute(const JMTTrajectory& traj,
                         const VehicleConfiguration& goalConf,
                         const double requestTime,
                         const TrackedVehicleMap& trackedVehicleMap,
                         const JMTTrajectoryEvaluator::Options& options) = 0;
  virtual ~CostFunctorBase() {}
};

/**
 * @brief Cost for time difference
 *
 * Penalizes trajectories that span a duration which is longer or shorter than
 * the duration requested.
 */
struct TimeDiffCost : public CostFunctorBase
{
  double Compute(const JMTTrajectory& traj,
                 const VehicleConfiguration& goalConf,
                 const double requestTime,
                 const TrackedVehicleMap& trackedVehicleMap,
                 const JMTTrajectoryEvaluator::Options& options) override;
};

/**
 * @brief      Cost for difference between goal S
 *
 * Penalizes trajectories whose s coordinate (and derivatives) differ from the
 * goal.
 */
struct SDiffCost : public CostFunctorBase
{
  double Compute(const JMTTrajectory& traj,
                 const VehicleConfiguration& goalConf,
                 const double requestTime,
                 const TrackedVehicleMap& trackedVehicleMap,
                 const JMTTrajectoryEvaluator::Options& options) override;
};

/**
 * @brief      Cost for difference between goal D
 *
 * Penalizes trajectories whose d coordinate (and derivatives) differ from the
 * goal.
 */
struct DDiffCost : public CostFunctorBase
{
  double Compute(const JMTTrajectory& traj,
                 const VehicleConfiguration& goalConf,
                 const double requestTime,
                 const TrackedVehicleMap& trackedVehicleMap,
                 const JMTTrajectoryEvaluator::Options& options) override;
};

/**
 * @brief      Binary cost function which penalizes collisions.
 */
struct CollisionCost : public CostFunctorBase
{
  double Compute(const JMTTrajectory& traj,
                 const VehicleConfiguration& goalConf,
                 const double requestTime,
                 const TrackedVehicleMap& trackedVehicleMap,
                 const JMTTrajectoryEvaluator::Options& options) override;
};

/**
 * @brief      Buffer distance cost
 *
 * Penalizes getting close to other vehicles.
 */
struct BufferCost : public CostFunctorBase
{
  double Compute(const JMTTrajectory& traj,
                 const VehicleConfiguration& goalConf,
                 const double requestTime,
                 const TrackedVehicleMap& trackedVehicleMap,
                 const JMTTrajectoryEvaluator::Options& options) override;
};

/**
 * @brief      Cost for the distance from the center of the target lane
 */
struct StaysOnRoadCost : public CostFunctorBase
{
  double Compute(const JMTTrajectory& traj,
                 const VehicleConfiguration& goalConf,
                 const double requestTime,
                 const TrackedVehicleMap& trackedVehicleMap,
                 const JMTTrajectoryEvaluator::Options& options) override;
};

/**
 * @brief      Speed limit cost
 */
struct ExceedsSpeedLimitCost : public CostFunctorBase
{
  double Compute(const JMTTrajectory& traj,
                 const VehicleConfiguration& goalConf,
                 const double requestTime,
                 const TrackedVehicleMap& trackedVehicleMap,
                 const JMTTrajectoryEvaluator::Options& options) override;
};

/**
 * @brief      Rewards high average speeds.
 */
struct EfficiencyCost : public CostFunctorBase
{
  double Compute(const JMTTrajectory& traj,
                 const VehicleConfiguration& goalConf,
                 const double requestTime,
                 const TrackedVehicleMap& trackedVehicleMap,
                 const JMTTrajectoryEvaluator::Options& options) override;
};

/**
 * @brief      Total accelaration cost
 */
struct TotalAccelCost : public CostFunctorBase
{

  double Compute(const JMTTrajectory& traj,
                 const VehicleConfiguration& goalConf,
                 const double requestTime,
                 const TrackedVehicleMap& trackedVehicleMap,
                 const JMTTrajectoryEvaluator::Options& options) override;
};

/**
 * @brief      Max accelaration cost
 */
struct MaxAccelCost : public CostFunctorBase
{
  double Compute(const JMTTrajectory& traj,
                 const VehicleConfiguration& goalConf,
                 const double requestTime,
                 const TrackedVehicleMap& trackedVehicleMap,
                 const JMTTrajectoryEvaluator::Options& options) override;
};

/**
 * @brief      Total jerk cost
 */
struct TotalJerkCost : public CostFunctorBase
{
  double Compute(const JMTTrajectory& traj,
                 const VehicleConfiguration& goalConf,
                 const double requestTime,
                 const TrackedVehicleMap& trackedVehicleMap,
                 const JMTTrajectoryEvaluator::Options& options) override;
};

/**
 * @brief      Max jerk cost
 */
struct MaxJerkCost : public CostFunctorBase
{
  double Compute(const JMTTrajectory& traj,
                 const VehicleConfiguration& goalConf,
                 const double requestTime,
                 const TrackedVehicleMap& trackedVehicleMap,
                 const JMTTrajectoryEvaluator::Options& options) override;
};

} // namespace costs

std::ostream&
operator<<(std::ostream& out, const costs::CostType& type);

} // namespace pathplanning
