#ifndef PATHPLANNING_CONFIGURATION_H
#define PATHPLANNING_CONFIGURATION_H

namespace pathplanning {

struct Configuration {
  static constexpr double TIME_STEP = 0.02;
  static constexpr double TIME_HORIZON = 1.0;
};

}  // namespace pathplanning

#endif
