#include "collision_checker.h"

namespace pathplanning {

double
CollisionChecker::GetMinDistance(const JMTTrajectory2d& traj,
                                 const Vehicle& vehicle,
                                 double maxTimeDuration,
                                 double timeStep)
{
  double minDist = std::numeric_limits<double>::infinity();
  double currTime = 0.0;
  while (currTime < maxTimeDuration) {
    currTime += timeStep;
    Matrix62d trajKinematics = traj(currTime);
    Matrix32d vehicleKinematics = vehicle.GetKinematics(currTime);
    double dist = (trajKinematics.row(0) - vehicleKinematics.row(0)).norm();
    if (dist < minDist) {
      minDist = dist;
    }
  }
  return minDist;
}

std::pair<int, double>
CollisionChecker::GetMinDistance(const JMTTrajectory2d& traj,
                                 const std::unordered_map<int, Vehicle>& vehicles,
                                 double maxTimeDuration,
                                 double timeStep)
{
  int minId;
  double minDist = std::numeric_limits<double>::infinity();
  for (const auto& [id, v] : vehicles) {
    double dist = GetMinDistance(traj, v, maxTimeDuration, timeStep);
    if (dist < minDist) {
      minDist = dist;
      minId = id;
    }
  }
  return { minId, minDist };
}

std::pair<int, double>
CollisionChecker::IsInCollision(const JMTTrajectory2d& traj,
                                const TrackedVehicleMap& trackedVehicleMap,
                                const Configuration& conf)
{
  const auto& [id, dist] =
    GetMinDistance(traj, trackedVehicleMap, traj.GetTime(), conf.trajectory.collisionCheckingTimeStep);
  double threshold = std::max(Vehicle::Size, conf.trajectory.collisionCheckingRadius);
  if (0.0 < dist and dist < threshold) {
    return { id, dist };
  }
  return { -1, std::numeric_limits<double>::infinity() };
}

} // namespace pathplanning
