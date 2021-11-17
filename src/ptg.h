#ifndef PATHPLANNING_PTG_H
#define PATHPLANNING_PTG_H

#include "jmt.h"
#include "json.hpp"
#include "tracker.h"
#include "traj_evaluator.h"

namespace pathplanning {

using namespace nlohmann;

/**
 * @brief      This class describes a polynomial trajectory generator.
 */
class PolynomialTrajectoryGenerator
{
public:
  explicit PolynomialTrajectoryGenerator(const Map& map, const Configuration& conf);
  virtual ~PolynomialTrajectoryGenerator() {}

  JMTTrajectory2d GenerataTrajectory(const Ego& ego, const TrackedVehicleMap& trackedVehicleMap);

  Matrix32d ComputeStartState(const Vehicle& ego,
                              const JMTTrajectory2d& prevTraj,
                              const Waypoints& prevPath,
                              int numPointsToPreserve);

private:
  const Configuration& _conf;
  const Map& _map;
  std::unique_ptr<JMTTrajectoryEvaluator> _pEvaluator;
};

}; // namespace pathplanning

#endif
