#ifndef HELPERS_H
#define HELPERS_H

#include <cmath>

#include <string>
#include <vector>
#include "common.h"

// for convenience
using std::string;
using std::vector;

namespace pathplanning {

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s);

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
inline constexpr double pi() { return M_PI; }

inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }

inline std::tuple<std::vector<double>, std::vector<double>>
ConverPathToXY(const Path &path) {
  std::vector<double> x;
  std::vector<double> y;
  for (const auto &point : path) {
    x.push_back(point[0]);
    y.push_back(point[1]);
  }
  return std::make_tuple(x, y);
}

inline Path
ConvertXYToPath(const std::vector<double> &x, const std::vector<double> &y)
{
  Path path;
  for (size_t i = 0; i < x.size(); ++i) {
    path.push_back({x[i], y[i]});
  }
  return path;
}

inline void PrintPathSpeed(const Path &path) {
  std::cout << "speed: ";
  for (size_t i = 1; i < path.size(); ++i) {
    const double d = std::sqrt(
      std::pow(path[i][0] - path[i - 1][0], 2) +
      std::pow(path[i][1] - path[i - 1][1], 2)
    );
    std::cout << d / 0.02 * 2.24 << ", ";
  }
  std::cout << std::endl;
}

inline void PrintPath(const Path &path) {
  std::cout << path << std::endl;
}


// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2);

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(
  double x, double y,
  const vector<double> &maps_x, const vector<double> &maps_y);

// Returns next waypoint of the closest waypoint
int NextWaypoint(
    double x, double y, double theta,
    const vector<double> &maps_x,
    const vector<double> &maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(
    double x, double y, double theta,
    const vector<double> &maps_x,
    const vector<double> &maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(
  double s, double d,
  const vector<double> &maps_s,
  const vector<double> &maps_x,
  const vector<double> &maps_y);

inline double GetDValueFromLandId(int laneId) {
  static constexpr double laneWidth = 4.0; // meter
  static constexpr double halfLaneWidth = 2.0; // meter
  return halfLaneWidth + laneWidth * static_cast<double>(laneId);
}

} // end of pathplanning

#endif  // HELPERS_H
