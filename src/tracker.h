#ifndef PATHPLANNING_TRACKER_H
#define PATHPLANNING_TRACKER_H

#include "configuration.h"
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

using TrackedVehicleMap = std::unordered_map<int, Vehicle>;

class Tracker
{
public:
  struct Options
  {
    std::array<double, 2> sdhorizon = Configuration::SD_HORIZON;
  };

  Tracker(const Map::ConstPtr& pMap)
    : _pMap(pMap)
  {}

  virtual ~Tracker() {}

  /**
   * @brief      Update tracking vechiles using new perception results
   *
   * @param[in]  perceptions  The new perceptions results
   */
  void Update(const Vehicle& ego, const Perceptions& perceptions);

  const TrackedVehicleMap& GetVehicles() const { return _trackedVehicleMap; }

  bool IsEmpty() const { return _trackedVehicleMap.empty(); }

  bool HasVehicle(const int id) const
  {
    return _trackedVehicleMap.count(id) != 0;
  }

  const Vehicle& GetVehicle(const int id) const
  {
    return _trackedVehicleMap.at(id);
  }

private:
  Options _options;
  Map::ConstPtr _pMap;
  TrackedVehicleMap _trackedVehicleMap;
};
} // namespace pathplanning

// IO functions

inline std::ostream&
operator<<(std::ostream& out,
           const pathplanning::TrackedVehicleMap& trackedVehicles)
{
  for (const auto& v : trackedVehicles) {
    out << fmt::format("{:2d}: {:s}\n", v.first, v.second);
  }
  return out;
}

#endif
