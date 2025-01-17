#ifndef PATHPLANNING_PTG_H
#define PATHPLANNING_PTG_H

#include <boost/optional.hpp>

#include "jmt.h"
#include "json.hpp"
#include "tracker.h"

namespace pathplanning {

using namespace nlohmann;

enum class LaneType
{
  kEgo,
  kLeft,
  kRight,
};

enum class LongitudinalManeuverType
{
  kFollowing,
  kCruising,
  kStopping,
};

enum class LateralManeuverType
{
  kLaneKeeping,
  kLeftLaneChanging,
  kRightLaneChanging,
};

/**
 * @brief      This class describes a polynomial trajectory generator.
 */
class PolynomialTrajectoryGenerator
{
public:
  explicit PolynomialTrajectoryGenerator(const Map& map, const Configuration& conf);
  virtual ~PolynomialTrajectoryGenerator() {}

  JMTTrajectory2d GenerateTrajectory(const Ego& ego, const TrackedVehicleMap& trackedVehicleMap);

private:
  class Impl;
  std::shared_ptr<Impl> _pImpl;

  const Map& _map;
  const Configuration& _conf;
};

std::ostream&
operator<<(std::ostream& out, const LongitudinalManeuverType& type);

std::ostream&
operator<<(std::ostream& out, const LateralManeuverType& type);

}; // namespace pathplanning

#endif
