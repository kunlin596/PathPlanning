#ifndef PATHPLANNING_CONFIGURATION_H
#define PATHPLANNING_CONFIGURATION_H

#include <array>
#include <fstream>

#include "json.hpp"
#include "math.h"

namespace pathplanning {

/**
 * @brief      This class describes a configuration for path planner.
 */
class Configuration
{
public:
  Configuration(){};
  Configuration(const std::string& filename) { Parse(filename); }

  int serverPort = 4567;

  std::string mode = "cpp";

  double simulatorTimeStep = 0.02;
  double nonEgoSearchRadius = 100.0;
  double maxVelocity = Mph2Mps(50.0);
  double maxAcceleration = 10.0;
  double maxJerk = 10.0;
  double averageAcceleration = 3.0;
  double averageJerk = 2.0;
  double collisionCheckingRadius = 7.0;

  void Parse(const std::string& filename);
};

} // namespace pathplanning

#endif
