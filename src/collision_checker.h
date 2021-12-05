#pragma once
#include "jmt.h"
#include "tracker.h"

namespace pathplanning {

struct CollisionChecker
{
  static double GetMinDistance(const JMTTrajectory2d& traj, const Vehicle& vehicle, double timeStep);

  static std::pair<int, double> GetMinDistance(const JMTTrajectory2d& traj,
                                               const std::unordered_map<int, Vehicle>& vehicles,
                                               double timeStep);

  static std::pair<int, double> IsInCollision(const JMTTrajectory2d& traj,
                                              const TrackedVehicleMap& trackedVehicleMap,
                                              const Configuration& conf);
};

} // namespace pathplanning
