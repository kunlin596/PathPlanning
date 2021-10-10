#ifndef COMMON_H
#define COMMON_H

#include <vector>
#include <iostream>
#include <array>
#include <cmath>
#include <boost/format.hpp>

namespace pathplanning {
///
/// Data Structures
///

struct SensorFusion {
  std::array<double, 2> velocity; ///< vector
  double speed; ///< magnitude of velocity, meters/s
  std::array<double, 2> frenetPose;

  SensorFusion(double vx, double vy, double s, double d)
    : velocity({vx, vy})
    , frenetPose({s, d})
  {
    speed = std::sqrt(velocity[0] * velocity[0] + velocity[1] * velocity[1]);
  }

  static std::vector<SensorFusion> Read(const std::vector<std::vector<double>> &data) {
    std::vector<SensorFusion> sensorFusions;
    for (size_t i = 0; i < data.size(); ++i) {
      sensorFusions.push_back(
        SensorFusion(
          data[i][3], data[i][4],
          data[i][5], data[i][6]
        )
      );
    }
    return sensorFusions;
  }
};

using SensorFusions = std::vector<SensorFusion>;

struct Lane {
  int id = -1;
  inline static const double kWidth = 4.0;
  double averageSpeed = 0.0;
  SensorFusions sensorFusions;

  Lane() {}
  Lane(int id): id(id) {}
};

struct Road {
  std::vector<Lane> lanes;

  int GetLandId(const double d) const {
    for (const auto &lane : lanes) {
      if ((lane.id * Lane::kWidth - Lane::kWidth / 2.0) < d and
           d < (lane.id * Lane::kWidth + Lane::kWidth / 2.0)) {
        return lane.id;
      }
    }
    return -1;
  }

  double GetLaneCenterDValue(const int laneId) const {
    const double d = laneId * Lane::kWidth + Lane::kWidth / 2.0;
    const std::array<double, 2> roadBoundary = {
      0, lanes.size() * Lane::kWidth + Lane::kWidth / 2.0
    };
    if (roadBoundary[0] < d and d < roadBoundary[1]) {
      return d;
    }
    return std::numeric_limits<double>::quiet_NaN();
  }
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

  Road road; ///< Assuming all roads have the same number of lanes (spec)
};

using Path = std::vector<std::array<double, 2>>;

enum class BehaviorState : uint8_t {
  kLaneKeeping = 0,
  kLeftLaneChangePreparation,
  kLeftLaneChange,
  kRightLaneChangePreparation,
  kRightLaneChange
};

struct Behavior {
  Lane lane;
  double targetSpeed;
  BehaviorState state;

  Behavior(
    Lane lane,
    double targetSpeed,
    BehaviorState state = BehaviorState::kLaneKeeping
  )
    : lane(lane)
    , targetSpeed(targetSpeed)
    , state(state)
  {}
};

struct CarState {
  Lane lane;
  std::array<double, 3> euclideanPose; ///< theta, x, y 
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
  {
    const double d = frenetPose[1];
    if (0 <= d and d < 4.0) {
      lane = Lane(0);
    } else if (4.0 <= d and d < 8.0) {
      lane = Lane(1);
    } else if (8.0 <= d and d < 12.0) {
      lane = Lane(2);
    }
  }
};

} // end of pathplanning

inline std::ostream& operator<< (std::ostream &out, const std::array<double, 2> &array) {
  out << boost::format("[%.3f, %.3f]") % array[0] % array[1];
  return out;
}

inline std::ostream& operator<< (std::ostream &out, const std::array<double, 3> &array) {
  out << boost::format("[%.3f, %.3f, %.3f]") % array[0] % array[1] % array[2];
  return out;
}

inline std::ostream& operator<< (std::ostream &out, const std::vector<std::array<double, 2>> &arr) {
  out << "[\n";
  for (const std::array<double, 2> &point : arr) {
    out << " " << point << "\n";
  }
  out << "]\n";
  return out;
}

inline std::ostream& operator<< (std::ostream &out, const pathplanning::BehaviorState &state) {
  using namespace pathplanning;
  switch (state) {
    case BehaviorState::kLaneKeeping: out << "kLaneKeeping"; return out;
    case BehaviorState::kLeftLaneChangePreparation: out << "kLeftLaneChangePreparation"; return out;
    case BehaviorState::kLeftLaneChange: out << "kLeftLaneChange"; return out;
    case BehaviorState::kRightLaneChangePreparation: out << "kRightLaneChangePreparation"; return out;
    case BehaviorState::kRightLaneChange: out << "kRightLaneChange"; return out;
    default: return out;
  }
}

inline std::ostream& operator<< (std::ostream &out, const pathplanning::CarState &state) {
  // FIXME: There are some name looking issue where the overloaded operator cannot be found and resolved in compilation error.
  out << boost::format("CarState=(euclideanPose=[%.3f, %.3f, %.3f], frenetPose=[%.3f, %.3f], speed=%.3f")
      % state.euclideanPose[0] % state.euclideanPose[1] % state.euclideanPose[2]
      % state.frenetPose[0] % state.frenetPose[1]
      % state.speed;
  return out;
}

inline std::ostream& operator<< (std::ostream &out, const pathplanning::Behavior &behavior) {
  // FIXME: There are some name looking issue where the overloaded operator cannot be found and resolved in compilation error.
  out << boost::format("Behavior=(laneId=%d, targetSpeed=%.3f)")
    % behavior.lane.id % behavior.targetSpeed;
  return out;
}

#endif

