#ifndef PATHPLANNING_PTG_H
#define PATHPLANNING_PTG_H
#include <memory>

#include "jmt.h"

namespace pathplanning {

class GoalSampler {
 public:
  explicit GoalSampler(const VehicleConfiguration &canonicalConf,
                       const std::array<double, 6> &sigmas);

  VehicleConfiguration Sample() const;

 private:
  std::array<std::unique_ptr<GaussianSampler1D<>>, 6> _samplers;
};

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
   * @param[in]  startParams      The start ego configuration
   * @param[in]  endParams        The end ego configuration
   * @param[in]  predictions      The non-ego vehicle perceived configuration
   * @param[in]  targetVehicleId  The target vehicle identifier
   * @param[in]  paramDelta       The parameter delta w.r.t. target vehicle
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
