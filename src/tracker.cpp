#include "tracker.h"

#include <iostream>
#include <set>

#include "configuration.h"

namespace pathplanning {

void Tracker::Update(const Vehicle &ego, const Perceptions &perceptions) {
  // TODO: Implementation can be simplified.
  const VehicleConfiguration &egoConf = ego.GetConfiguration(0.0);
  std::array<double, 2> egoSD = {egoConf.sPos, egoConf.dPos};

  std::set<int> idToBeIgnored;
  for (const auto &perception : perceptions) {
    double dist = GetDistance(perception.second.sd, egoSD);
    if (dist > Configuration::NONEGO_SEARCH_RADIUS) {
      idToBeIgnored.insert(perception.first);
    }
  }

  // Naively remove disappered vehicle.
  std::vector<int> idToBeRemoved;
  for (const auto &vehicleData : _trackedVehicleMap) {
    if (perceptions.count(vehicleData.first) == 0) {
      idToBeRemoved.push_back(vehicleData.first);
    }
  }

  // SPDLOG_DEBUG("received new perceptions: {}", perceptions);
  for (auto &id : idToBeRemoved) {
    _trackedVehicleMap.erase(id);
  }

  // Process new vehicles;
  for (const auto &perception : perceptions) {
    if (idToBeIgnored.count(perception.first)) {
      if (_trackedVehicleMap.count(perception.first)) {
        _trackedVehicleMap.erase(perception.first);
      }
      continue;
    }

    const int id = perception.first;
    _trackedVehicleMap[id] =
        Vehicle::CreateFromPerception(_pMap, perception.second);
    // SPDLOG_INFO("Append new observation to {}, in lane {}", id,
    //             Map::GetLaneId(perception.second.sd[1]));
  }
  // SPDLOG_INFO("Size of tracked vehicles {}", _trackedVehicles.size());
}

}  // namespace pathplanning
