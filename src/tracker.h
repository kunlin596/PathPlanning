#ifndef PATHPLANNING_TRACKER_H
#define PATHPLANNING_TRACKER_H

#include "configuration.h"
#include "log.h"
#include "map.h"
#include "perception.h"
#include "vehicle.h"
// #include "spline.h"  // For generating prediction

namespace pathplanning {

using TrackedVehicleMap = std::unordered_map<int, Vehicle>;


/**
 * @brief      This class describes a non-ego vehicle tracker.
 *
 * This class is responsible for creating predictions and keep track of the
 * non-ego vehicles.
 */
class Tracker
{
public:
  struct Options
  {
    double nonEgoSearchRadius = 30.0;
    double timeStep = 0.02;
    int numMeasurementsToTrack = 30;
    Options(const Configuration& conf)
    {
      timeStep = conf.timeStep;
      nonEgoSearchRadius = conf.tracker.nonEgoSearchRadius;
      numMeasurementsToTrack = conf.tracker.numMeasurementsToTrack;
    }
  };

  Tracker(const Map::ConstPtr& pMap, const Options& options)
    : _pMap(pMap)
    , _options(options)
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

std::ostream&
operator<<(std::ostream& out, const TrackedVehicleMap& trackedVehicles);

} // namespace pathplanning

// IO functions

#endif
