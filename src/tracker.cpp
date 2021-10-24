#include "tracker.h"

#include <iostream>
#include <set>

#include "configuration.h"

namespace pathplanning {

Predictions Tracker::TrackedVehicle::GeneratePredictions(
    const double time) const {
  // TODO: Use more sophisticated method like spline fitting later.
  Predictions predictions;
  if (observations.empty()) {
    return predictions;
  }

  const Vehicle &lastObservation = observations[observations.size() - 1];
  // Generate numPredictions + 1 predictions, the first one the current
  // configuration.
  int numPredictions = static_cast<int>(time / Configuration::TIME_STEP) + 1;
  // SPDLOG_INFO("numPredictions={}", numPredictions);

  predictions[id] = std::vector<Vehicle>(numPredictions);
  for (int i = 0; i < numPredictions; ++i) {
    predictions[id][i] = Vehicle(
        id, lastObservation.GetConfiguration(i * Configuration::TIME_STEP));
    // SPDLOG_INFO("predictions[{}][{}]={}", id, i, predictions[id][i]);
  }
  return predictions;
}

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
  for (const auto &vehicleData : _trackedVehicles) {
    if (perceptions.count(vehicleData.first) == 0) {
      idToBeRemoved.push_back(vehicleData.first);
    }
  }

  // SPDLOG_DEBUG("received new perceptions: {}", perceptions);
  for (auto &id : idToBeRemoved) {
    _trackedVehicles.erase(id);
  }

  // Process new vehicles;
  for (const auto &perception : perceptions) {
    if (idToBeIgnored.count(perception.first)) {
      if (_trackedVehicles.count(perception.first)) {
        _trackedVehicles.erase(perception.first);
      }
      continue;
    }

    const int id = perception.first;
    _trackedVehicles[id].id = id;
    _trackedVehicles[id].observations.push_back(
        Vehicle::CreateFromPerception(_pMap, perception.second));
    // SPDLOG_INFO("Append new observation to {}, in lane {}", id,
    //             Map::GetLaneId(perception.second.sd[1]));
  }
  // SPDLOG_INFO("Size of tracked vehicles {}", _trackedVehicles.size());
}

Predictions Tracker::GeneratePredictions() const {
  Predictions predictions;
  // For each tracked vehicle, generate a set of predictions per time step
  for (const auto &trackedVehicle : _trackedVehicles) {

    Predictions pred =
        trackedVehicle.second.GeneratePredictions(Configuration::TIME_HORIZON);
    // SPDLOG_INFO("id={}, pred.size()={}", trackedVehicle.first, pred.size());
    // SPDLOG_INFO("Generated {} predictions for {}",
    //             pred[trackedVehicle.first].size(), trackedVehicle.first);
    predictions.insert(pred.begin(), pred.end());
  }
  return predictions;
}

}  // namespace pathplanning
