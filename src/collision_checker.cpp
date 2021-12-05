#include "collision_checker.h"
#include "log.h"

namespace pathplanning {

double
CollisionChecker::GetMinDistance(const JMTTrajectory2d& traj, const Vehicle& vehicle, double timeStep)
{
  double minDist = std::numeric_limits<double>::infinity();
  // NOTE: We need to constraint the collision checking time, since the kinematics of non-ego are only approximations.
  double trajTime = std::min(traj.GetTime(), 1.0);
  for (double currTime = 0.0; currTime < (trajTime + 1e-6); currTime += timeStep) {
    Vector2d trajLonLat = traj(currTime).topRows<1>();
    Vector2d vehicleLonLat = vehicle.GetKinematics(currTime).topRows<1>();

    double dist = (trajLonLat - vehicleLonLat).norm();

    if (dist < 2.0) {
      SPDLOG_INFO(" --- id={:d}, time={:3.f}, dist={:.3f}, {}, {}",
                  vehicle.GetId(),
                  currTime,
                  dist,
                  trajLonLat.transpose(),
                  vehicleLonLat.transpose());
    }
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
