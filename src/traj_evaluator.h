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
  kEfficiency,
  kTotalAccel,
  kTotalJerk,
};

struct CostFunctorBase;

} // namesapce costs

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
    std::string driverProfileName;
    std::unordered_map<costs::CostType, double> driverProfile;

    Options() {}
    Options(const Configuration& conf);
  };

  JMTTrajectoryEvaluator(const Configuration& conf)
    : _conf(conf)
    , _options(conf)
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
  double Evaluate(const JMTTrajectory2d& traj,
                  const Matrix32d& goalConf,
                  const double requestTime,
                  const TrackedVehicleMap& trackedVehicleMap);

private:
  std::unordered_map<costs::CostType, std::shared_ptr<costs::CostFunctorBase>> _funcPtrs; ///< Cost funtions
  Configuration _conf;
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
  virtual double operator()(const JMTTrajectory2d& traj,
                            const Matrix32d& goalConf,
                            const double requestTime,
                            const TrackedVehicleMap& trackedVehicleMap,
                            const Configuration& conf) = 0;

  virtual ~CostFunctorBase() {}
};

#ifndef DEFINE_COST_FUNCTOR
#define DEFINE_COST_FUNCTOR(NAME)                                                                                      \
  struct NAME : public CostFunctorBase                                                                                 \
  {                                                                                                                    \
    virtual double operator()(const JMTTrajectory2d& traj,                                                             \
                              const Matrix32d& goalConf,                                                               \
                              const double requestTime,                                                                \
                              const TrackedVehicleMap& trackedVehicleMap,                                              \
                              const Configuration& conf) override;                                                     \
  }
#endif

/**
 * @brief Cost for time difference
 *
 * Penalizes trajectories that span a duration which is longer or shorter than
 * the duration requested.
 */
DEFINE_COST_FUNCTOR(TimeDiffCost);

/**
 * @brief      Cost for difference between goal S
 *
 * Penalizes trajectories whose s coordinate (and derivatives) differ from the
 * goal.
 */
DEFINE_COST_FUNCTOR(SDiffCost);

/**
 * @brief      Cost for difference between goal D
 *
 * Penalizes trajectories whose d coordinate (and derivatives) differ from the
 * goal.
 */
DEFINE_COST_FUNCTOR(DDiffCost);

/**
 * @brief      Buffer distance cost
 *
 * Penalizes getting close to other vehicles.
 */
DEFINE_COST_FUNCTOR(BufferCost);

/**
 * @brief      Cost for the distance from the center of the target lane
 */
DEFINE_COST_FUNCTOR(StaysOnRoadCost);

/**
 * @brief      Rewards high average speeds.
 */
DEFINE_COST_FUNCTOR(EfficiencyCost);

/**
 * @brief      Total accelaration cost
 */
DEFINE_COST_FUNCTOR(TotalAccelCost);

/**
 * @brief      Total jerk cost
 */
DEFINE_COST_FUNCTOR(TotalJerkCost);

std::ostream&
operator<<(std::ostream& out, const CostType& type);

} // namespace costs
} // namespace pathplanning
