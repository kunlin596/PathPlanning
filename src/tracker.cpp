#include "tracker.h"

#include <iostream>
#include <set>

#include "configuration.h"

namespace pathplanning {

void
Tracker::Update(const Vehicle& ego, const Perceptions& perceptions)
{
  // TODO: Implementation can be simplified.
  const VehicleConfiguration& egoConf = ego.GetConfiguration(0.0);
  std::array<double, 2> egoSD = { egoConf.sPos, egoConf.dPos };

  // Filter out the vehicles that are out of search range
  std::set<int> idToBeIgnored;
  for (const auto& perception : perceptions) {
    double dist = GetDistance(perception.second.sd, egoSD);
    if (dist > _options.nonEgoSearchRadius) {
      idToBeIgnored.insert(perception.first);
    }
  }

  // Naively remove disappered vehicle from tracked vehicle.
  std::vector<int> idToBeRemoved;
  for (const auto& vehicleData : _trackedVehicleMap) {
    if (perceptions.count(vehicleData.first) == 0) {
      idToBeRemoved.push_back(vehicleData.first);
    }
  }

  // SPDLOG_DEBUG("received new perceptions: {}", perceptions);
  for (auto& id : idToBeRemoved) {
    _trackedVehicleMap.erase(id);
  }

  // Process new vehicles
  for (const auto& perception : perceptions) {
    if (idToBeIgnored.count(perception.first)) {
      if (_trackedVehicleMap.count(perception.first)) {
        _trackedVehicleMap.erase(perception.first);
      }
      continue;
    }

    const int id = perception.first;
    if (_trackedVehicleMap.count(id)) {
      _trackedVehicleMap[id].Update(perception.second);
      SPDLOG_INFO("Update existing id {:2d}");
    } else {
      _trackedVehicleMap[id] =
        Vehicle::CreateFromPerception(_pMap,
                                      perception.second,
                                      _options.timeStep,
                                      _options.numMeasurementsToTrack);
      SPDLOG_INFO("Create new id {:2d}");
    }
    SPDLOG_INFO("In lane {:1d}, add to id {:3d}, distance in sd: "
                "[{:7.3f}, {:7.3f}]",
                Map::GetLaneId(perception.second.sd[1]),
                id,
                perception.second.sd[0] - egoConf.sPos,
                perception.second.sd[1] - egoConf.dPos);
  }
  SPDLOG_INFO("Size of tracked vehicles {}", _trackedVehicleMap.size());
}
std::ostream&
operator<<(std::ostream& out, const TrackedVehicleMap& trackedVehicles)
{
  for (const auto& v : trackedVehicles) {
    out << fmt::format("{:2d}: {:s}\n", v.first, v.second);
  }
  return out;
}

} // namespace pathplanning
