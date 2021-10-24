#ifndef PATHPLANNING_TRACKER_H
#define PATHPLANNING_TRACKER_H

#include "log.h"
#include "map.h"
#include "perception.h"
#include "vehicle.h"
// #include "spline.h"  // For generating prediction

namespace pathplanning {

/**
 * @brief      This class describes a non-ego vehicle tracker.
 *
 * This class is responsible for creating predictions and keep track of the
 * non-ego vehicles.
 */

using Predictions = std::unordered_map<int, std::vector<Vehicle>>;

class Tracker {
 public:
  struct TrackedVehicle {
    int id = -1;
    std::vector<Vehicle> observations;  ///< Keep track of all observations

    Predictions GeneratePredictions(const double time = 1.0) const;
  };

  Tracker(const Map::ConstPtr &pMap) : _pMap(pMap) {}
  virtual ~Tracker() {}

  /**
   * @brief      Update tracking vechiles using new perception results
   *
   * @param[in]  perceptions  The new perceptions results
   */
  void Update(const Perceptions &perceptions);

  Predictions GeneratePredictions() const;

  const std::unordered_map<int, TrackedVehicle> &GetVehicles() const {
    return _trackedVehicles;
  }

  bool IsEmpty() const { return _trackedVehicles.empty(); }

  bool HasVehicle(const int id) const {
    return _trackedVehicles.count(id) != 0;
  }

  const TrackedVehicle &GetTrackedVehicle(const int id) const {
    return _trackedVehicles.at(id);
  }

 private:
  Map::ConstPtr _pMap;
  std::unordered_map<int, TrackedVehicle> _trackedVehicles;
};
}  // namespace pathplanning

// IO functions

inline std::ostream &operator<<(
    std::ostream &out, const pathplanning::Tracker::TrackedVehicle &v) {
  return out << fmt::format("{:d}: {\n{:s}}\n", v.id, v.observations);
}

inline std::ostream &operator<<(std::ostream &out,
                                const pathplanning::Predictions &predictions) {
  for (const auto &p : predictions) {
    out << fmt::format("{:d}: {:s}\n", p.first, p.second);
  }
  return out;
}

#endif
