#include "perception.h"

#include <set>

namespace pathplanning {

void PerceptionManager::Update(
    const std::vector<std::vector<double>> &rawdata) {
  // NOTE: Can be optimized

  // Collect the current perceived car ids
  std::set<int> curIds;
  for (const auto &perception : _perceptions) {
    curIds.insert(perception.first);
  }

  std::set<int> newIds;
  for (size_t i = 0; i < rawdata.size(); ++i) {
    newIds.insert(rawdata[i][0]);
  }

  // Remove the car id which can not be seen in current measurements.
  // This is OK because it's simulation and the sensor measurements are assumed
  // to be perfect.
  for (const auto &curId : curIds) {
    if (newIds.count(curId) == 0) {
      _perceptions.erase(curId);
    }
  }

  // Record new perception results.
  for (size_t i = 0; i < rawdata.size(); ++i) {
    const std::vector<double> &perception = rawdata[i];
    _perceptions[perception[0]].push_back(
        Perception(perception[0], perception[1], perception[2], perception[3],
                   perception[4], perception[5], perception[6]));
  }
}

};  // namespace pathplanning
