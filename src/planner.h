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
  const std::array<double, 2> &endPathFrenetPose,
  int numPreservedWaypoints = 2
);

} // end of pathplanning
#endif
