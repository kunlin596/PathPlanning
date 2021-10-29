#ifndef PATHPLANNING_PATH_H
#define PATHPLANNING_PATH_H

#include <array>
#include <tuple>
#include <vector>

#include "log.h"

namespace pathplanning {

using Waypoint = std::array<double, 2>;
using Waypoints = std::vector<Waypoint>;

class Path
{
public:
  static std::tuple<std::vector<double>, std::vector<double>>
  ConvertWaypointsToXY(const Waypoints& waypoints);

  static Waypoints ConvertXYToWaypoints(const std::vector<double>& x,
                                        const std::vector<double>& y);
  friend std::ostream& operator<<(std::ostream& out,
                                  const pathplanning::Waypoints& waypoints);
};

std::ostream&
operator<<(std::ostream& out, const pathplanning::Waypoints& waypoints);

} // namespace pathplanning

// IO functions

#endif
