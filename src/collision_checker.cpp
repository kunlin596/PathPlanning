#include "collision_checker.h"

namespace pathplanning {

double
CollisionChecker::GetMinDistance(const JMTTrajectory2d& traj,
                                 const Vehicle& vehicle,
                                 double timeStep)
{
  double minDist = std::numeric_limits<double>::infinity();
  for (double currTime = 0.0; currTime < (traj.GetTime() + 1e-6); currTime += timeStep) {
    Vector2d trajLonLat = traj(currTime).topRows<1>();
    Vector2d vehicleLonLat = vehicle.GetKinematics(currTime).topRows<1>();
    double dist = (trajLonLat - vehicleLonLat).norm();
    if (dist < minDist) {
      minDist = dist;
    }
  }
  return minDist;
}

std::pair<int, double>
CollisionChecker::GetMinDistance(const JMTTrajectory2d& traj,
                                 const std::unordered_map<int, Vehicle>& vehicles,
                                 double timeStep)
{
  int minId = -1;
  double minDist = std::numeric_limits<double>::infinity();
  for (const auto& [id, v] : vehicles) {
    double dist = GetMinDistance(traj, v, timeStep);
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
  const auto& [id, dist] = GetMinDistance(traj, trackedVehicleMap, conf.trajectory.collisionCheckingTimeStep);
  double threshold = std::max(Vehicle::Size / 2.0, conf.trajectory.collisionCheckingRadius);
  if (0.0 < dist and dist < threshold) {
    return { id, dist };
  }
  return { -1, std::numeric_limits<double>::infinity() };
}

} // namespace pathplanning
