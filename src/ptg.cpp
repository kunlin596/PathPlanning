#include "ptg.h"

#include "log.h"

namespace pathplanning {

GoalSampler::GoalSampler(const VehicleConfiguration& canonicalConf,
                         const std::array<double, 6>& sigmas)
{
  for (size_t i = 0; i < 6; ++i) {
    _samplers[i] =
      std::make_unique<GaussianSampler1D<>>(canonicalConf.At(i), sigmas[i]);
  }
}

VehicleConfiguration
GoalSampler::Sample() const
{
  return VehicleConfiguration(_samplers[0]->Sample(),
                              _samplers[1]->Sample(),
                              _samplers[2]->Sample(),
                              _samplers[3]->Sample(),
                              _samplers[4]->Sample(),
                              _samplers[5]->Sample());
}

std::pair<Waypoints, JMTTrajectory2D>
PolynomialTrajectoryGenerator::GeneratePath(
  const Vehicle& startState,
  const Vehicle& goalState,
  const TrackedVehicleMap& trackedVehicleMap,
  const int numPointsToBeGenerated,
  const double targetExecutionTime)
{
  VehicleConfiguration startConf = startState.GetConfiguration();

  Waypoints waypoints(numPointsToBeGenerated);

  JMTTrajectory2D bestTrajectory;
  VehicleConfiguration* pBestGoal;

  if (_options.use) {
    //
    // Generate perturbed goals
    //

    std::vector<VehicleConfiguration> goals;
    std::vector<int> goalTimes;
    const double sampleTimeStart =
      targetExecutionTime - 4 * targetExecutionTime * 0.01;
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

      sampleTime += _options.sampleTimeStep;
    }

    double minCost = std::numeric_limits<double>::max();
    for (size_t i = 0; i < goals.size(); ++i) {
      JMTTrajectory2D trajectory =
        JMT::ComputeTrajectory(startConf, goals[i], goalTimes[i]);

      double cost = _pEvaluator->Evaluate(
        trajectory, goals[i], targetExecutionTime, trackedVehicleMap);
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

  SPDLOG_INFO("start={}, goal={} targetExecutionTime={:5.3f}, "
              "numPointsToBeGenerated={}",
              startConf,
              targetExecutionTime,
              numPointsToBeGenerated);

  for (size_t i = 0; i < numPointsToBeGenerated; ++i) {
    VehicleConfiguration conf = bestTrajectory(_options.timeStep * i);
    waypoints[i] = _pMap->GetXY(conf.sPos, conf.dPos);
  }

  return std::make_pair(waypoints, bestTrajectory);
}

void
PolynomialTrajectoryGenerator::ComputeStartState(const Vehicle& ego,
                                                 const JMTTrajectory2D& prevTraj,
                                                 const Waypoints& prevPath,
                                                 const Waypoint& endPrevPathSD,
                                                 double& executedTime,
                                                 Vehicle& startState,
                                                 int& numPoints)
{
  if (prevPath.empty()) {
    startState = ego;
    return;
  }

  executedTime = prevPath.size() * _options.timeStep;

  // clang-format off
  startState = Vehicle(ego.GetId(), prevTraj(executedTime));
  numPoints = _options.numPoints - prevPath.size();

  // clang-format on
  auto diff = startState.GetConfiguration() - ego.GetConfiguration();

  SPDLOG_INFO("executedTime={:7.3f}, diff=[{:7.3f}, {:7.3f}]",
              executedTime,
              diff[0],
              diff[3]);
}

} // namespace pathplanning
