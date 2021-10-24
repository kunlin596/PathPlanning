#ifndef PATHPLANNING_CONFIGURATION_H
#define PATHPLANNING_CONFIGURATION_H

#include <array>

namespace pathplanning {

struct Configuration {
  static constexpr double TIME_STEP = 0.02;
  static constexpr double TIME_HORIZON = 1.0;
  static constexpr std::array<double, 2> SD_HORIZON = {30, 10};
  static constexpr double NONEGO_SEARCH_RADIUS = 10.0;
};

}  // namespace pathplanning

#endif
