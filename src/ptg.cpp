#include "ptg.h"

#include "log.h"

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

Waypoints PolynomialTrajectoryGenerator::GeneratePath(
    const Vehicle &startState, const Vehicle &goalState,
    const Predictions &predictions, const double t) const {
  //
  // Generate perturbed goals
  //

  SPDLOG_INFO("t={}, sampleTimeStart={}", t, _options.goalTimeSampleStep);
  std::vector<VehicleConfiguration> goals;
  std::vector<int> goalTimes;

  SPDLOG_INFO("t={}, sampleTimeStart={}", t, _options.goalTimeSampleStep);
  const double sampleTimeStart = t - 4 * _options.goalTimeSampleStep;
  double sampleTime = sampleTimeStart;

  for (size_t i = 0; i < 4; ++i) {
    VehicleConfiguration targetConf = goalState.GetConfiguration(sampleTime);

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
    JMTTrajectory trajectory = JMT::ComputeTrajectory(
        startState.GetConfiguration(), goals[i], goalTimes[i]);

    // FIXME: Fix delta format
    double cost = _pEvaluator->Validate(trajectory, t, predictions);
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
    Waypoint sd = func(_options.timeStep * i);
    waypoints[i] = _pMap->GetXY(sd[0], sd[1]);
  }
  return waypoints;
}

}  // namespace pathplanning
