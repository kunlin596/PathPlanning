#include "configuration.h"

#define PARSE_PROPERTY(NAME)                                                                                           \
  if (j.count("##NAME")) {                                                                                             \
    NAME = j["##NAME"];                                                                                                \
  }

namespace pathplanning {
void
Configuration::Parse(const std::string& filename)
{
  using namespace nlohmann;

  json j;
  if (not filename.empty()) {
    std::ifstream ifs(filename);
    ifs >> j;
    std::cout << j.dump(2) << std::endl;
  }

  PARSE_PROPERTY(serverPort);
  PARSE_PROPERTY(mode);
  PARSE_PROPERTY(simulatorTimeStep);
  PARSE_PROPERTY(nonEgoSearchRadius);

  PARSE_PROPERTY(maxVelocity);
  PARSE_PROPERTY(maxAcceleration);
  PARSE_PROPERTY(maxJerk);
  PARSE_PROPERTY(averageAcceleration);
  PARSE_PROPERTY(averageJerk);
  PARSE_PROPERTY(collisionCheckingRadius);

  PARSE_PROPERTY(maxLaneChangingTriggerDistance);
  PARSE_PROPERTY(maxStoppingTriggerDistance);
  PARSE_PROPERTY(maxFollowTriggerDistance);

  PARSE_PROPERTY(lonCruiseMinTime);
  PARSE_PROPERTY(lonCruiseMaxTime);
  PARSE_PROPERTY(lonCruiseNumTimeSteps);
  PARSE_PROPERTY(lonCruiseMinSddot);
  PARSE_PROPERTY(lonCruiseMaxSddot);
  PARSE_PROPERTY(lonCruiseNumSddotSteps);
  PARSE_PROPERTY(lonCruiseTimeWeight);
  PARSE_PROPERTY(lonCruisePosWeight);
  PARSE_PROPERTY(lonCruiseJerkWeight);
  PARSE_PROPERTY(lonCruiseEfficiencyWeight);
}
} // namespace pathplanning
