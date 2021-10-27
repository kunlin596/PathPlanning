#include "path.h"

namespace pathplanning {

std::tuple<std::vector<double>, std::vector<double>>
Path::ConvertWaypointsToXY(const Waypoints& waypoints)
{
  using std::vector;
  vector<double> x(waypoints.size()), y(waypoints.size());
  for (size_t i = 0; i < waypoints.size(); ++i) {
    const auto& p = waypoints[i];
    x[i] = p[0];
    y[i] = p[1];
  }
  return std::make_pair(x, y);
}

Waypoints
Path::ConvertXYToWaypoints(const std::vector<double>& x,
                           const std::vector<double>& y)
{
  Waypoints waypoints;
  for (size_t i = 0; i < x.size(); ++i) {
    waypoints.push_back({ x[i], y[i] });
  }
  return waypoints;
}

} // namespace pathplanning
