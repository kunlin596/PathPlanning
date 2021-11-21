#include "tracker.h"

#include <iostream>
#include <set>

#include "configuration.h"

namespace pathplanning {

void
Tracker::Update(const Ego& ego, const Perceptions& perceptions)
{
  // TODO: Implementation can be simplified.
  std::array<double, 2> egoSD = { ego._kinematics(0, 0), ego._kinematics(0, 1) };

  // Filter out the vehicles that are out of search range
  std::set<int> idToBeIgnored;
  for (const auto& [perceptionId, perception] : perceptions) {
    double dist = GetDistance(perception.sd, egoSD);
    if (dist > _conf.tracker.nonEgoSearchRadius) {
      idToBeIgnored.insert(perceptionId);
    }
  }

  // Naively remove disappeared vehicle from tracked vehicle.
  std::vector<int> idToBeRemoved;
  for (const auto& [vehicleId, vehicleData] : _trackedVehicleMap) {
    if (perceptions.count(vehicleId) == 0) {
      idToBeRemoved.push_back(vehicleId);
    }
  }

  // SPDLOG_DEBUG("received new perceptions: {:s}", perceptions);
  for (auto& id : idToBeRemoved) {
    _trackedVehicleMap.erase(id);
  }

  // Process new vehicles
  for (const auto& [perceptionId, perception] : perceptions) {

    if (idToBeIgnored.count(perceptionId) and _trackedVehicleMap.count(perceptionId)) {
      _trackedVehicleMap.erase(perceptionId);
      continue;
    }

    int laneId = Map::GetLaneId(perception.sd[1]);
    _trackedVehicleMap[perceptionId] = perception.GetVehicle();
  }
  SPDLOG_TRACE("Size of tracked vehicles {}", _trackedVehicleMap.size());
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
