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

std::ostream&
operator<<(std::ostream& out, const pathplanning::Waypoints& waypoints)
{
  out << std::string("[\n");
  for (const auto& p : waypoints) {
    out << fmt::format("{:s},\n", p);
  }
  return out << std::string("]\n");
}

} // namespace pathplanning
