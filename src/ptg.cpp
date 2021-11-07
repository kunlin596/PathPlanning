#include "ptg.h"

#include "collision_checker.h"
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
    double maxTime = proposal.GetTime() + timeRangeSingleSide;

    // goalKinematics.push_back(proposal(0.0).block<3, 2>(0, 0));
    // goalTimes.push_back(proposal.GetTime());

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

    std::vector<JMTTrajectory2d> trajs;
    for (size_t i = 0; i < goalKinematics.size(); ++i) {
      Matrix62d conditions;
      conditions.block<3, 2>(0, 0) = startKinematics;
      conditions.block<3, 2>(3, 0) = goalKinematics[i];
      JMTTrajectory2d trajectory = JMT::Solve2d(conditions, goalTimes[i]);

      // SPDLOG_DEBUG("conditions=\n{}", conditions.format(HeavyFmt));
      if (not trajectory.IsValid(_map, _conf)) {
        SPDLOG_TRACE("trajectory invalid {}", i);
        continue;
      }
      const auto& [id, dist] = CollisionChecker::IsInCollision(trajectory, trackedVehicleMap, _conf);
      if (id != -1) {
        Eigen::Vector3d sKinematics = trackedVehicleMap.at(id).GetKinematics(0.0).block<3, 1>(0, 0);
        SPDLOG_TRACE(
          "trajectory, in collision {} with {}, vehicleS={}", i, id, sKinematics.transpose().format(HeavyFmt));
        continue;
      }

      trajs.push_back(trajectory);
      // if (CollisionChecker::IsInCollision(trajectory, trackedVehicleMap, _conf)) {
      //   continue;
      // }

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
    if (trajs.empty()) {
      SPDLOG_WARN("No valid trajectories found!");
    } else {
      bestTrajectory = trajs[trajs.size() / 2];
    }
  } else {
    const auto& [id, dist] = (CollisionChecker::IsInCollision(proposal, trackedVehicleMap, _conf));
    if (id != -1) {
      SPDLOG_WARN("Collision detected, id={}, closest dist={:7.3f}, kinematics={}",
                  id,
                  dist,
                  trackedVehicleMap.at(id).GetKinematics(0.0).topRows<1>().format(HeavyFmt));
    }
    bestTrajectory = proposal;
  }

  SPDLOG_DEBUG("bestTrajectory={}", bestTrajectory);

  //
  // Evaluate trajectory into waypoints
  //

  int numPointsToBeGenerated = static_cast<int>(std::round(bestTrajectory.GetTime() / _conf.simulator.timeStep));

  Waypoints waypoints(numPointsToBeGenerated);
  for (size_t i = 0; i < numPointsToBeGenerated; ++i) {
    Matrix62d trajKinematics = bestTrajectory(_conf.simulator.timeStep * i);
    waypoints[i] = _map.GetXY(trajKinematics(0, 0), trajKinematics(0, 1));
  }

  // log["bestTrajectory"] = bestTrajectory.Dump();

  std::tie(log["next_x"], log["next_y"]) = Path::ConvertWaypointsToXY(waypoints);

  return std::make_pair(waypoints, bestTrajectory);
}

Matrix32d
PolynomialTrajectoryGenerator::ComputeStartState(const Vehicle& ego,
                                                 const JMTTrajectory2d& prevTraj,
                                                 const Waypoints& prevPath,
                                                 int numPointsToPreserve)
{
  if (prevPath.empty()) {
    return ego.GetKinematics(0.0);
  }

  double executedTime = (_conf.numPoints - prevPath.size() + numPointsToPreserve) * _conf.simulator.timeStep;
  SPDLOG_DEBUG("executedTime={}", executedTime);
  return prevTraj(executedTime).topRows<3>();
}

} // namespace pathplanning
