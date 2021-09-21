#ifndef COMMON_H
#define COMMON_H

#include <vector>
#include <iostream>
#include <array>
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

using ProbeData = std::vector<std::vector<double>>;

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

struct BehaviorState {
  int laneId;
  double speed;

  BehaviorState(int laneId, double speed)
    : laneId(laneId)
    , speed(speed)
  {}
};

}

#endif

