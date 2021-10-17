#include "perception.h"

namespace pathplanning {

void PerceptionManager::Update(
    const std::vector<std::vector<double>> &rawdata) {
  _perceptions.clear();
  for (size_t i = 0; i < rawdata.size(); ++i) {
    const std::vector<double> &perception = rawdata[i];
    _perceptions.push_back(
        Perception(perception[0], perception[1], perception[2], perception[3],
                   perception[4], perception[5], perception[6]));
  }
}

};  // namespace pathplanning
