#include "tracker.h"

#include <iostream>

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

  predictions[id] = std::vector<Vehicle>(numPredictions);
  for (int i = 0; i < numPredictions; ++i) {
    predictions[id][i] = Vehicle(
        id, lastObservation.GetConfiguration(i * Configuration::TIME_STEP));
  }
  return predictions;
}

void Tracker::Update(const Perceptions &perceptions) {
  // TODO: Implementation can be simplified.

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
    std::cout << id << std::endl;
  }

  // Process new vehicles;
  for (const auto &perception : perceptions) {
    const int id = perception.first;
    _trackedVehicles[id].observations.push_back(
        Vehicle::CreateFromPerception(_pMap, perception.second));
  }
  SPDLOG_DEBUG("_trackedVehicles={:s}", _trackedVehicles);
}

Predictions Tracker::GeneratePredictions() const {
  Predictions predictions;
  // For each tracked vehicle, generate a set of predictions per time step
  for (const auto &trackedVehicle : _trackedVehicles) {
    auto pred =
        trackedVehicle.second.GeneratePredictions(Configuration::TIME_HORIZON);
    predictions.insert(pred.begin(), pred.end());
  }
  return predictions;
}

}  // namespace pathplanning
