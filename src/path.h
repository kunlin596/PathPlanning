#ifndef PATHPLANNING_PATH_H
#define PATHPLANNING_PATH_H

#include <array>
#include <tuple>
#include <vector>

namespace pathplanning {

using Waypoint = std::array<double, 2>;
using Waypoints = std::vector<Waypoint>;

class Path {
 public:
  std::tuple<std::vector<double>, std::vector<double>> ConverWaypointsToXY(
      const Waypoints& waypoints);

  inline Waypoints ConvertXYToWaypoints(const std::vector<double>& x,
                                        const std::vector<double>& y);
};

}  // namespace pathplanning

#endif
