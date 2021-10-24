#ifndef PATHPLANNING_CONFIGURATION_H
#define PATHPLANNING_CONFIGURATION_H

#include <array>

namespace pathplanning {

struct Configuration {
  static constexpr int SERVER_PORT = 4567;
  static constexpr double TIME_STEP = 0.02;
  static constexpr double TIME_HORIZON = 1.0;
  static constexpr double NUM_POINTS = TIME_HORIZON / TIME_STEP;
  static constexpr std::array<double, 2> SD_HORIZON = {30, 10};
  static constexpr double NONEGO_SEARCH_RADIUS = 30.0;
};

}  // namespace pathplanning

#endif
