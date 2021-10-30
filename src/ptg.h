#ifndef PATHPLANNING_PTG_H
#define PATHPLANNING_PTG_H
#include <memory>

#include "jmt.h"
#include "tracker.h"
#include "traj_evaluator.h"

namespace pathplanning {

/**
 * @brief      This class describes a goal sampler.
 *
 * Goal sampler is trying to find the best goal w.r.t. target canonical
 * configuration.
 */
class GoalSampler
{
public:
  explicit GoalSampler(const VehicleConfiguration& canonicalConf,
                       const std::array<double, 6>& sigmas);

  VehicleConfiguration Sample() const;

private:
  std::array<std::unique_ptr<GaussianSampler1D<>>, 6> _samplers;
};

/**
 * @brief      This class describes a polynomial trajectory generator.
 */
class PolynomialTrajectoryGenerator
{
public:
  struct Options
  {
    int numPoints = 50;
    int numSamples = 10;
    double timeStep = 0.02;

    bool use = false;
    double sampleTimeStep = 0.5;
    std::array<double, 6> sampleSigmas = { 10.0, 1.0, 2.0, 1.0, 1.0, 1.0 };

    int numMeasurementsToTrack = 30;
    double nonEgoSearchRadius = 30.0;

    JMTTrajectoryEvaluator::Options trajEvaluation;

    Options() {}

    Options(const Configuration& conf)
      : trajEvaluation(conf)
    {
      numMeasurementsToTrack = conf.tracker.numMeasurementsToTrack;
      nonEgoSearchRadius = conf.tracker.nonEgoSearchRadius;

      numPoints = conf.numPoints;
      timeStep = conf.timeStep;

      use = conf.goalSampler.use;
      numSamples = conf.goalSampler.numSamples;
      sampleTimeStep = conf.goalSampler.sampleTimeStep;
      sampleSigmas = conf.goalSampler.sampleSigmas;
    };
  };

  PolynomialTrajectoryGenerator(const Map::ConstPtr& pMap,
                                const Options& options)
    : _pMap(pMap)
    , _options(options)
  {
    _pEvaluator =
      std::make_unique<JMTTrajectoryEvaluator>(_options.trajEvaluation);
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
   */
  std::pair<Waypoints, JMTTrajectory> GeneratePath(
    const Vehicle& startState,
    const Vehicle& goalState,
    const TrackedVehicleMap& trackedVehicleMap,
    const double targetExecutionTime = 1.0);

  Vehicle ComputeStartState(const Vehicle& ego,
                            const JMTTrajectory& prevTraj,
                            const Waypoints& prevPath,
                            const Waypoint& endPrevPathSD);

private:
  Map::ConstPtr _pMap;
  Options _options;
  std::unique_ptr<VehicleConfiguration>
    _pPrevGoal; ///< Cache the previous goal for continuous planning
  std::unique_ptr<JMTTrajectoryEvaluator> _pEvaluator;
};

}; // namespace pathplanning

#endif
