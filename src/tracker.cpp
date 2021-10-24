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
  int numPredictions = static_cast<int>(time / Configuration::TIME_STEP);
  predictions[id] = std::vector<Vehicle>(numPredictions);

  // Generate numPredictions + 1 predictions, the first one the current
  // configuration.
  for (int i = 0; i < numPredictions + 1; ++i) {
    predictions[id][i] = Vehicle(
        id, lastObservation.GetConfiguration(i * Configuration::TIME_STEP));
    std::cout << predictions[id][i] << std::endl;
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

  for (auto &id : idToBeRemoved) {
    _trackedVehicles.erase(id);
  }

  // Process new vehicles;
  decltype(_trackedVehicles) newVehicles;
  for (auto &vehicleData : _trackedVehicles) {
    const int id = vehicleData.first;
    if (perceptions.count(id) != 0) {
      _trackedVehicles[id].observations.push_back(
          Vehicle::CreateFromPerception(_pMap, perceptions.at(id)));
    } else {
      newVehicles[id].observations = {
          Vehicle::CreateFromPerception(_pMap, perceptions.at(id))};
    }
  }
  _trackedVehicles.insert(newVehicles.begin(), newVehicles.end());
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
