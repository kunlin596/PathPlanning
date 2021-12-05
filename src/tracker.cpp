#include "tracker.h"

#include <iostream>
#include <set>

#include "configuration.h"

namespace pathplanning {

void
Tracker::Update(const Ego& ego, const Perceptions& perceptions)
{
  // TODO: Implementation can be simplified.
  std::array<double, 2> egoSd = { ego._kinematics(0, 0), ego._kinematics(0, 1) };

  _trackedVehicleMap.clear();

  // Filter out the vehicles that are out of search range
  Perceptions filteredPerceptions;
  for (const auto& [perceptionId, perception] : perceptions) {
    double dist = GetDistance(perception.sd, egoSd);
    if (dist < _conf.tracker.nonEgoSearchRadius) {
      filteredPerceptions[perceptionId] = perception;
    }
  }

  for (const auto& [perceptionId, perception] : filteredPerceptions) {
    int laneId = Map::GetLaneId(perception.sd[1]);
    _trackedVehicleMap[perceptionId] = perception.GetVehicle();
  }
  SPDLOG_TRACE("Size of tracked vehicles {}.", _trackedVehicleMap.size());
}

std::ostream&
operator<<(std::ostream& out, const TrackedVehicleMap& trackedVehicles)
{
  for (const auto& [id, vehicle] : trackedVehicles) {
    out << fmt::format("{:2d}: {:s}\n", id, vehicle);
  }
  return out;
}

} // namespace pathplanning
