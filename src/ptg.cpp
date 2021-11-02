#include "ptg.h"

#include "goalsampler.h"
#include "log.h"
#include "utils.h"

#include <boost/assert.hpp>

namespace pathplanning {

PolynomialTrajectoryGenerator::PolynomialTrajectoryGenerator(const Map& map, const Configuration& conf)
  : _map(map)
  , _conf(conf)
{}

std::pair<Waypoints, JMTTrajectory2d>
PolynomialTrajectoryGenerator::GeneratePath(const JMTTrajectory2d& proposal,
                                            const TrackedVehicleMap& trackedVehicleMap,
                                            json& log)
{

  Matrix32d startKinematics = proposal.GetStartCond();

  // log["proposal"] = proposal.Dump();

  // log["trackedVehicles"] = json::array();
  // for (const auto v : trackedVehicleMap) {
  //   log["trackedVehicles"].push_back(v.second.Dump());
  // }

  // log["allTraj"] = json::array();
  // log["allGoals"] = json::array();
  // log["allCosts"] = json::array();

  JMTTrajectory2d bestTrajectory;

  if (_conf.goalSampler.use) {
    //
    // Generate perturbed goals
    //

    std::vector<Matrix32d> goalKinematics;
    std::vector<double> goalTimes;

    double timeRangeSingleSide = _conf.goalSampler.numTimeSteps * _conf.goalSampler.sampleTimeStep;
    double minTime = std::max(0.5, proposal.GetTime() - timeRangeSingleSide);
    double maxTime = std::min(3.0, proposal.GetTime() + timeRangeSingleSide);

    double sampleTime = minTime;
    while (sampleTime < maxTime) {
      Matrix32d targetKinematics = proposal(sampleTime).block<3, 2>(0, 0);

      goalKinematics.push_back(targetKinematics);
      goalTimes.push_back(sampleTime);

      GoalSampler sampler(targetKinematics, _conf.goalSampler.sampleSigmas);
      for (int i = 0; i < _conf.goalSampler.numSamplesPerTimeStep; ++i) {
        goalKinematics.push_back(sampler.Sample());
        goalTimes.push_back(sampleTime);
      }

      sampleTime += _conf.goalSampler.sampleTimeStep;
    }

    //
    // Evaluate all goals by generating the trajectory and select best one
    //

    double minCost = std::numeric_limits<double>::max();
    using namespace nlohmann;
    json j;

    for (size_t i = 0; i < goalKinematics.size(); ++i) {
      Matrix62d conditions;
      conditions.block<3, 2>(0, 0) = startKinematics;
      conditions.block<3, 2>(3, 0) = goalKinematics[i];
      JMTTrajectory2d trajectory = JMT::Solve2D(conditions, goalTimes[i]);

      // log["allTraj"].push_back(trajectory.Dump());
      // log["allGoals"].push_back(goals[i].Dump());

      // double cost = _pEvaluator->Evaluate(trajectory, goals[i], proposal.GetTime(), trackedVehicleMap);
      // log["allCosts"].push_back(cost);

      // if (cost > 1e3) {
      //   continue;
      // }

      // SPDLOG_DEBUG("    - {:3d} cost={:7.3f}", i, cost);
      // if (cost < minCost) {
      //   minCost = cost;
      //   bestTrajectory = trajectory;
      // }
    }
  } else {
    bestTrajectory = proposal;
  }

  SPDLOG_INFO("bestTrajectory={}", bestTrajectory);

  //
  // Evaluate trajectory into waypoints
  //

  int numPointsToBeGenerated = static_cast<int>(std::round(bestTrajectory.GetTime() / _conf.timeStep));

  Waypoints waypoints(numPointsToBeGenerated);
  for (size_t i = 0; i < numPointsToBeGenerated; ++i) {
    Matrix62d trajKinematics = bestTrajectory(_conf.timeStep * i);
    waypoints[i] = _map.GetXY(trajKinematics(0, 0), trajKinematics(0, 1));
  }

  log["bestTrajectory"] = bestTrajectory.Dump();

  std::tie(log["next_x"], log["next_y"]) = Path::ConvertWaypointsToXY(waypoints);

  return std::make_pair(waypoints, bestTrajectory);
}

void
PolynomialTrajectoryGenerator::ComputeStartState(const Vehicle& ego,
                                                 const JMTTrajectory2d& prevTraj,
                                                 const Waypoints& prevPath,
                                                 const Waypoint& endPrevPathSD,
                                                 double& executedTime,
                                                 Matrix32d& startState,
                                                 int& numPoints)
{
  if (prevPath.empty()) {
    startState = ego.GetKinematics(0.0);
    return;
  }

  if (prevPath.empty()) {
    executedTime = 0.0;
  } else {
    executedTime = prevTraj.GetTime() - (prevPath.size() * _conf.timeStep);
  }

  BOOST_ASSERT(_conf.numPoints > prevPath.size());
  numPoints = _conf.numPoints - prevPath.size();
}

} // namespace pathplanning
