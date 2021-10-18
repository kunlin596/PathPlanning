#include "ptg.h"

namespace pathplanning {

GoalSampler::GoalSampler(const double s, const double sSigma, const double d,
                         const double dSigma) {
  _sSampler = std::make_unique<GaussianSampler1D<>>(s, sSigma);
  _dSampler = std::make_unique<GaussianSampler1D<>>(d, dSigma);
}

std::array<double, 2> GoalSampler::Sample() const {
  return {_sSampler->Sample(), _dSampler->Sample()};
}

Waypoints PolynomialTrajectoryGenerator::Generate(
    const std::array<double, 6> &sParams, const std::array<double, 6> &dParams,
    const double t) const {
  GoalSampler sampler(sParams[3], _options.sSamplerSigma, dParams[3],
                      _options.dSamplerSigma);

  Waypoints bestWaypoints;
  for (size_t i = 0; i < _options.numSamples; ++i) {
    // Copy the goal and modify it's sd values to create new goal.
    std::array<double, 6> sParamsSampled = sParams;
    std::array<double, 6> dParamsSampled = dParams;
    std::array<double, 2> sampledGoal = sampler.Sample();
    sParamsSampled[3] = sampledGoal[0];
    dParamsSampled[3] = sampledGoal[1];
    JMTTrajectory trajectory =
        JMT::ComputeTrajectory(sParamsSampled, dParamsSampled, t);

    auto func = trajectory.sdFunc;
    size_t numPoints = static_cast<size_t>(t / _options.timeStep);
    Waypoints waypoints(numPoints);
    for (size_t j = 0; j < numPoints; ++i) {
      waypoints[j] = func(_options.timeStep * j);
    }
    // TODO: Validate the path
    bestWaypoints = waypoints;
  }
  return bestWaypoints;
}

}  // namespace pathplanning
