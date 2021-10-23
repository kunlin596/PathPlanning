#ifndef PATHPLANNING_PTG_H
#define PATHPLANNING_PTG_H
#include <memory>

#include "jmt.h"

namespace pathplanning {

/**
 * @brief      This class describes a goal sampler.
 *
 * Goal sampler is trying to find the best goal w.r.t. target canonical
 * configuration.
 */
class GoalSampler {
 public:
  explicit GoalSampler(const VehicleConfiguration &canonicalConf,
                       const std::array<double, 6> &sigmas);

  VehicleConfiguration Sample() const;

 private:
  std::array<std::unique_ptr<GaussianSampler1D<>>, 6> _samplers;
};

/**
 * @brief      This class describes a polynomial trajectory generator.
 */
class PolynomialTrajectoryGenerator {
 public:
  struct Options {
    uint32_t numSamples = 10;
    double timeStep = 0.02;
    std::array<double, 6> sampleSigmas;
    double goalTimeSampleStep = 0.5;
  };

  PolynomialTrajectoryGenerator(
      const Options &options, const costs::CostWeightMapping &costWeightMapping)
      : _options(options) {
    _pValidator = std::make_unique<JMTTrajectoryValidator>(costWeightMapping);
  }

  /**
   * @brief      Generate drivable trajectory (a set of waypoints)
   *
   * PTG uses start and end configurations of from the behavior planner to
   * generate trajectories. `targetVehicleId` is the id of the target vehicle
   * that we are setting out trajectory relative to.
   *
   * `deltaConf` is the configuration offset w.r.t. the target vehicle.
   *
   * `targetVehicleId` is the id of the targeting vehicle in the perception
   * result list. When `-1` is provided, it means we don't want to plan w.r.t.
   * any vehicle, the trajectory will fallback to lane keeping.
   *
   * So if at time 5 the target vehicle will be at [100, 10, 0, 0, 0, 0] and
   * delta is [-10, 0, 0, 4, 0, 0], then our goal state for t = 5 will be [90,
   * 10, 0, 4, 0, 0]. This would correspond to a goal of "follow 10 meters
   * behind and 4 meters to the right of target vehicle".
   *
   * `t` is the desired time at which we will be at the goal (relative to now as
   * t=0).
   *
   * `perceptions` is a map of {id: Vehicle}. The vehicle represents the current
   * state of the vehicle, and can be used to predict future states.
   *
   *
   * @param[in]  startParams      The start ego configuration
   * @param[in]  endParams        The end ego configuration
   * @param[in]  predictions      The non-ego vehicle perceived configuration
   * @param[in]  targetVehicleId  The target vehicle identifier
   * @param[in]  deltaConf        The configuration delta w.r.t. target vehicle
   * @param[in]  t                Target trajectory execution time
   *
   * @return     { description_of_the_return_value }
   */
  Waypoints Generate(const VehicleConfiguration &startConf,
                     const VehicleConfiguration &endConf,
                     const std::unordered_map<int, Vehicle> &perceptions,
                     const int targetVehicleId,
                     const VehicleConfiguration &deltaConf,
                     const double t) const;

 private:
  Options _options;
  std::unique_ptr<JMTTrajectoryValidator> _pValidator;
};

};  // namespace pathplanning

#endif
