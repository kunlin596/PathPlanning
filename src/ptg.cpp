#include "ptg.h"

namespace pathplanning {

GoalSampler::GoalSampler(const VehicleConfiguration &canonicalConf,
                         const std::array<double, 6> &sigmas) {
  for (size_t i = 0; i < 6; ++i) {
    _samplers[i] =
        std::make_unique<GaussianSampler1D<>>(canonicalConf.At(i), sigmas[i]);
  }
}

VehicleConfiguration GoalSampler::Sample() const {
  return VehicleConfiguration(_samplers[0]->Sample(), _samplers[1]->Sample(),
                              _samplers[2]->Sample(), _samplers[3]->Sample(),
                              _samplers[4]->Sample(), _samplers[5]->Sample());
}

Waypoints PolynomialTrajectoryGenerator::Generate(
    const VehicleConfiguration &startConf, const VehicleConfiguration &endConf,
    const std::unordered_map<int, Vehicle> &perceptions,
    const int targetVehicleId, const VehicleConfiguration &deltaConf,
    const double t) const {
  if (perceptions.count(targetVehicleId) == 0) {
    throw std::runtime_error("Cannot find target vehicle");
  }

  const Vehicle &targetVehicle = perceptions.at(targetVehicleId);

  //
  // Generate perturbed goals
  //

  std::vector<VehicleConfiguration> goals;
  std::vector<int> goalTimes;

  const double sampleTimeStart = t - 4 * _options.goalTimeSampleStep;
  double sampleTime = sampleTimeStart;

  for (size_t i = 0; i < 4; ++i) {
    VehicleConfiguration targetConf =
        targetVehicle.GetConfiguration(sampleTime) + deltaConf;

    goals.push_back(targetConf);

    GoalSampler sampler(targetConf, _options.sampleSigmas);
    for (size_t j = 0; j < _options.numSamples; ++j) {
      goals.push_back(sampler.Sample());
      goalTimes.push_back(sampleTime);
    }

    sampleTime += _options.goalTimeSampleStep;
  }

  //
  // Evaluate all goals by generating the trajectory and select best one
  //

  double minCost = std::numeric_limits<double>::max();
  JMTTrajectory bestTrajectory;
  for (size_t i = 0; i < goals.size(); ++i) {
    JMTTrajectory trajectory =
        JMT::ComputeTrajectory(startConf, goals[i], goalTimes[i]);

    // FIXME: Fix delta format
    double cost =
        _pEvaluator->Validate(trajectory, targetVehicleId, 0.0, t, perceptions);
    if (cost < minCost) {
      minCost = cost;
      bestTrajectory = trajectory;
    }
  }

  //
  // Evaluate trajectory into waypoints
  //

  auto func = bestTrajectory.sdFunc;
  size_t numPoints = static_cast<size_t>(t / _options.timeStep);
  Waypoints waypoints(numPoints);
  for (size_t i = 0; i < numPoints; ++i) {
    waypoints[i] = func(_options.timeStep * i);
  }
  return waypoints;
}

}  // namespace pathplanning
