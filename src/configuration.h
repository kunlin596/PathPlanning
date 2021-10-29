#ifndef PATHPLANNING_CONFIGURATION_H
#define PATHPLANNING_CONFIGURATION_H

#include <array>

#include "math.h"

namespace pathplanning {

struct Configuration
{
  static constexpr int SERVER_PORT = 4567;

  static constexpr double SPEED_LIMIT = Mph2Mps(50.0);
  static constexpr double TIME_STEP = 0.02;   ///< Seconds
  static constexpr double TIME_HORIZON = 2.0; ///< Time horizon for prediction
  static constexpr int NUM_POINTS = static_cast<int>(TIME_HORIZON / TIME_STEP);
  static constexpr std::array<double, 2> SD_HORIZON = { 30, 10 };

  static constexpr double NONEGO_SEARCH_RADIUS = 30.0;
  static constexpr int NUM_MEASUREMENTS_TO_TRACK = 30;

  struct TrajectoryEvaluation
  {
    static constexpr std::array<double, 3> SIGMA_S = {
      10.0,
      1.0,
      2.0
    }; ///< Sigma for s pos, s vel, s acc sampling

    static constexpr std::array<double, 3> SIGMA_D = {
      1.0,
      1.0,
      1.0
    }; ///< Sigma for d pos, d vel, d acc sampling

    static constexpr double MAX_JERK = 10.0;                 ///< m/s^4
    static constexpr double MAX_ACC = 10.0;                  ///< m/s^3
    static constexpr double EXPECTED_JERK_IN_ONE_SEC = 2.0;  ///< m/s^2
    static constexpr double EXPECTED_ACC_IN_ONE_SEC = 1.0;   ///< m/s^3
    static constexpr double COLLISION_CHECKING_RADIUS = 1.5; ///< meter
  };
};

} // namespace pathplanning

#endif
