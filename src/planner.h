#ifndef PLANNER_H
#define PLANNER_H

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/Dense"
#include "Eigen-3.3/Eigen/QR"
#include "common.h"

namespace pathplanning {
///
/// Helper Functions
///

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

///
/// Path Generators
///

/**
 * Dummy path generator, generate a straight line path
 *
 * This generator simply generates 50 evenly distributed waypoints
 */
Path GeneratePath(const CarState &carState);

/**
 * Lane following path generator
 */
Path GeneratePath(const CarState &carState, const NaviMap &naviMap);

/**
 * Generate path using spline line fitting
 */
Path GeneratePath(
  int targetLaneId,
  double speedReference,
  const CarState &carState,
  const Path &prevPath,
  const NaviMap &naviMap,
  int numPreservedWaypoints = 2
);

} // end of pathplanning
#endif
