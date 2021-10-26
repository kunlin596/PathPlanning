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

std::pair<Waypoints, JMTTrajectory> PolynomialTrajectoryGenerator::GeneratePath(
    const Vehicle &startState, const Vehicle &goalState,
    const Predictions &predictions, const double targetExecutionTime) {
  VehicleConfiguration startConf = startState.GetConfiguration();
  const int numPointsToBeGenerated =
      std::min(static_cast<int>(targetExecutionTime / _options.timeStep),
               Configuration::NUM_POINTS);

  Waypoints waypoints(numPointsToBeGenerated);

  JMTTrajectory bestTrajectory;
  VehicleConfiguration *pBestGoal;

  if (_options.useGoalSampler) {
    //
    // Generate perturbed goals
    //

    std::vector<VehicleConfiguration> goals;
    std::vector<int> goalTimes;
    const double sampleTimeStart =
        targetExecutionTime - 4 * _options.goalTimeSampleStep;
    double sampleTime = sampleTimeStart;

    //
    // Evaluate all goals by generating the trajectory and select best one
    //

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

    double minCost = std::numeric_limits<double>::max();
    for (size_t i = 0; i < goals.size(); ++i) {
      JMTTrajectory trajectory =
          JMT::ComputeTrajectory(startConf, goals[i], goalTimes[i]);

      double cost = _pEvaluator->Evaluate(trajectory, goals[i],
                                          targetExecutionTime, predictions);
      if (cost < minCost) {
        minCost = cost;
        pBestGoal = &goals[i];
        bestTrajectory = trajectory;
      }
    }
  } else {
    bestTrajectory = JMT::ComputeTrajectory(
        startConf, goalState.GetConfiguration(), targetExecutionTime);
  }

  //
  // Evaluate trajectory into waypoints
  //

  SPDLOG_DEBUG(
      "start={}, targetExecutionTime={:7.3f}, numPointsToBeGenerated={}, "
      "waypoints.size()={}",
      startConf, targetExecutionTime, numPointsToBeGenerated, waypoints.size());

  for (size_t i = 0; i < numPointsToBeGenerated; ++i) {
    VehicleConfiguration conf = bestTrajectory(_options.timeStep * i);
    waypoints[i] = _pMap->GetXY(conf.sPos, conf.dPos);
  }

  return std::make_pair(waypoints, bestTrajectory);
}

Vehicle PolynomialTrajectoryGenerator::ComputeStartState(
    const Vehicle &ego, const JMTTrajectory &prevTraj,
    const Waypoints &prevPath, const Waypoint &endPrevPathSD) {
  if (prevPath.empty()) {
    return ego;
  }
  double executedTime =
      (Configuration::NUM_POINTS - prevPath.size()) * Configuration::TIME_STEP;

  // clang-format off
  VehicleConfiguration result = prevTraj(executedTime);

  // clang-format on
  auto diff = result - ego.GetConfiguration();

  SPDLOG_DEBUG("executedTime={:7.3f}, diff={}, {}", executedTime, diff[0],
               diff[3]);

  return Vehicle(ego.GetId(), result);
}

}  // namespace pathplanning
