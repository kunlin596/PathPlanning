#ifndef PLANNER_H
#define PLANNER_H

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/Dense"
#include "Eigen-3.3/Eigen/QR"
#include <iostream>
#include <vector>
#include <boost/format.hpp>

namespace pathplanning {

///
/// Data Structures
///

struct CarState {
  std::array<double, 3> euclideanPose; ///< x, y, theta
  std::array<double, 2> frenetPose; ///< s, d
  double speed; ///< car speed, in MPH

  CarState(
    const std::array<double, 3> &euclideanPose,
    const std::array<double, 2> &frenetPose,
    double speed
  )
    : euclideanPose(euclideanPose)
    , frenetPose(frenetPose)
    , speed(speed)
  {}
};

struct NaviMap {
  // Euclidean position
  std::vector<double> x;
  std::vector<double> y;
  // Frenet s value
  std::vector<double> s;
  // Normal vector w.r.t. x, y
  std::vector<double> dx;
  std::vector<double> dy;
};

using Path = std::vector<std::array<double, 2>>;

inline std::ostream& operator<< (std::ostream &out, const std::array<double, 2> &array) {
  out << (boost::format("[%.3f, %.3f]") % array[0] % array[1]).str();
  return out;
}

inline std::ostream& operator<< (std::ostream &out, const std::array<double, 3> &array) {
  out << (boost::format("[%.3f, %.3f, %.3f]") % array[0] % array[1] % array[2]).str();
  return out;
}

inline std::ostream& operator<< (std::ostream &out, const pathplanning::CarState &state) {
  out << "CarState=(euclideanPose="
      << state.euclideanPose
      << ", frenetPose="
      << state.frenetPose
      << ", speed=" << state.speed << ")";
  return out;
}

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
  int currLaneId,
  int targetLandId,
  const CarState &carState,
  const Path &prevPath,
  const NaviMap &naviMap,
  int numPreservedWaypoints = 2,
  double speedReference = 80.0
);

} // end of pathplanning
#endif
