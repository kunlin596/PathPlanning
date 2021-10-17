#include "path.h"

namespace pathplanning {

std::tuple<std::vector<double>, std::vector<double>> Path::ConverWaypointsToXY(
    const Waypoints& waypoints) {
  std::vector<double> x;
  std::vector<double> y;
  for (const auto& waypoint : waypoints) {
    x.push_back(waypoint[0]);
    y.push_back(waypoint[1]);
  }
  return std::make_tuple(x, y);
}

Waypoints Path::ConvertXYToWaypoints(const std::vector<double>& x,
                                     const std::vector<double>& y) {
  Waypoints waypoints;
  for (size_t i = 0; i < x.size(); ++i) {
    waypoints.push_back({x[i], y[i]});
  }
  return waypoints;
}

}  // namespace pathplanning
