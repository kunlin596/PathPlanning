#include "ptg.h"

#include <random>

#include "jmt.h"

namespace pathplanning {
Waypoints PolynomialTrajectoryGenerator::Generate(
    const std::array<double, 6> &sParams, const std::array<double, 6> &dParams,
    const double t) const {

  std::random_device rd{};
  std::mt19937 gen{rd()};

  Waypoints bestWaypoints;
  // For now, only sample the target location
  std::normal_distribution<> sSampler(sParams[3], _options.sSamplerSigma);
  std::normal_distribution<> dSampler(dParams[3], _options.dSamplerSigma);
  for (size_t i = 0; i < _options.numSamples; ++i) {
    std::array<double, 6> sParamsSampled = sParams;
    std::array<double, 6> dParamsSampled = dParams;
    sParamsSampled[3] = sSampler(gen);
    dParamsSampled[3] = dSampler(gen);
    SDFunctor func = JMT::Solve2D(sParamsSampled, dParamsSampled, t);
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
