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
                         const TrackedVehicleMap& trackedVehicleMap) = 0;
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
                 const TrackedVehicleMap& trackedVehicleMap) override;
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
                 const TrackedVehicleMap& trackedVehicleMap) override;
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
                 const TrackedVehicleMap& trackedVehicleMap) override;
};

/**
 * @brief      Binary cost function which penalizes collisions.
 */
struct CollisionCost : public CostFunctorBase
{
  double Compute(const JMTTrajectory& traj,
                 const VehicleConfiguration& goalConf,
                 const double requestTime,
                 const TrackedVehicleMap& trackedVehicleMap) override;
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
                 const TrackedVehicleMap& trackedVehicleMap) override;
};

/**
 * @brief      Cost for the distance from the center of the target lane
 */
struct StaysOnRoadCost : public CostFunctorBase
{
  double Compute(const JMTTrajectory& traj,
                 const VehicleConfiguration& goalConf,
                 const double requestTime,
                 const TrackedVehicleMap& trackedVehicleMap) override;
};

/**
 * @brief      Speed limit cost
 */
struct ExceedsSpeedLimitCost : public CostFunctorBase
{
  double Compute(const JMTTrajectory& traj,
                 const VehicleConfiguration& goalConf,
                 const double requestTime,
                 const TrackedVehicleMap& trackedVehicleMap) override;
};

/**
 * @brief      Rewards high average speeds.
 */
struct EfficiencyCost : public CostFunctorBase
{
  double Compute(const JMTTrajectory& traj,
                 const VehicleConfiguration& goalConf,
                 const double requestTime,
                 const TrackedVehicleMap& trackedVehicleMap) override;
};

/**
 * @brief      Total accelaration cost
 */
struct TotalAccelCost : public CostFunctorBase
{
  double Compute(const JMTTrajectory& traj,
                 const VehicleConfiguration& goalConf,
                 const double requestTime,
                 const TrackedVehicleMap& trackedVehicleMap) override;
};

/**
 * @brief      Max accelaration cost
 */
struct MaxAccelCost : public CostFunctorBase
{
  double Compute(const JMTTrajectory& traj,
                 const VehicleConfiguration& goalConf,
                 const double requestTime,
                 const TrackedVehicleMap& trackedVehicleMap) override;
};

/**
 * @brief      Total jerk cost
 */
struct TotalJerkCost : public CostFunctorBase
{
  double Compute(const JMTTrajectory& traj,
                 const VehicleConfiguration& goalConf,
                 const double requestTime,
                 const TrackedVehicleMap& trackedVehicleMap) override;
};

/**
 * @brief      Max jerk cost
 */
struct MaxJerkCost : public CostFunctorBase
{
  double Compute(const JMTTrajectory& traj,
                 const VehicleConfiguration& goalConf,
                 const double requestTime,
                 const TrackedVehicleMap& trackedVehicleMap) override;
};

} // namespace costs

/**
 * @brief      This class describes a jmt trajectory evaluator.
 *
 * Given cost weight mapping, create the cost functions defined in the mapping
 * and sum up all of the cost w.r.t. give weight.
 */
class JMTTrajectoryEvaluator
{
public:
  JMTTrajectoryEvaluator(const costs::CostWeightMapping& costWeightMapping =
                           costs::CostWeightMapping())
    : _costWeightMapping(costWeightMapping)
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
};

std::ostream&
operator<<(std::ostream& out, const pathplanning::costs::CostType& type);

} // namespace pathplanning
