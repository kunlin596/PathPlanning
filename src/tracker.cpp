#include "tracker.h"

namespace pathplanning {

void Tracker::Update(const Perceptions &perceptions) {
  // TODO: Implementation can be simplified.
  std::unordered_map<int, Perception> newPerceptionMap;

  for (const auto &perception : perceptions) {
    newPerceptionMap[perception.id] = perception;
  }

  // Naively remove disappered vehicle.
  std::vector<int> idToBeRemoved;
  for (const auto &vehicleData : _trackedVehicles) {
    if (newPerceptionMap.count(vehicleData.first) == 0) {
      idToBeRemoved.push_back(vehicleData.first);
    }
  }

  for (auto &id : idToBeRemoved) {
    _trackedVehicles.erase(id);
  }

  // Process new vehicles;
  decltype(_trackedVehicles) newVehicles;
  for (auto &vehicleData : _trackedVehicles) {
    const int id = vehicleData.first;
    if (newPerceptionMap.count(id) != 0) {
      _trackedVehicles[id].observations.push_back(
          Vehicle::CreateFromPerception(_pMap, newPerceptionMap[id]));
    } else {
      newVehicles[id].observations = {
          Vehicle::CreateFromPerception(_pMap, newPerceptionMap[id])};
    }
  }
  _trackedVehicles.insert(newVehicles.begin(), newVehicles.end());
}

}  // namespace pathplanning
