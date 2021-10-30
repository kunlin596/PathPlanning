#ifndef PATHPLANNING_CONFIGURATION_H
#define PATHPLANNING_CONFIGURATION_H

#include <array>
#include <fstream>

#include "json.hpp"
#include "math.h"

namespace pathplanning {

class Configuration
{
public:
  explicit Configuration(const std::string& filename) { Parse(filename); }

  int serverPort = 4567;

  double speedLimit = Mph2Mps(50.0);
  double timeStep = 0.02;   ///< Seconds
  double timeHorizon = 2.0; ///< Time horizon for prediction
  int numPoints = static_cast<int>(timeHorizon / timeStep);
  std::array<double, 2> sdHorizon = { 30, 10 };

  struct
  {
    double nonEgoSearchRadius = 30.0;
    int numMeasurementsToTrack = 30;
  } tracker;

  struct
  {
    double maxJerk = 10.0;                ///< m/s^3
    double maxAcc = 10.0;                 ///< m/s^2
    double expectedAccInOneSec = 2.0;     ///< m/s^2
    double expectedJerkInOneSec = 1.0;    ///< m/s^3
    double collisionCheckingRadius = 1.5; ///< meter
    std::array<double, 6> evalSigmas = {
      10.0, 1.0, 2.0, 1.0, 1.0, 1.0
    }; ///< Sigmas for [s pos, s vel, s acc, d pos, d vel, d acc] sampling

  } trajectoryEvaluation;

  std::string driverProfileName;
  struct
  {
    double timeDiff = 1.0;
    double sDiff = 1.0;
    double dDiff = 1.0;
    double collision = 1.0;
    double buffer = 1.0;
    double staysOnRoad = 1.0;
    double exceedsSpeedLimit = 1.0;
    double efficiency = 1.0;
    double totalAccel = 1.0;
    double maxAccel = 1.0;
    double totalJerk = 1.0;
    double maxJerk = 1.0;
  } driverProfile;

  struct
  {
    bool use = true;
    int numSamples = 20;
    double sampleTimeStep = 0.5;
    std::array<double, 6> sampleSigmas = {
      10.0, 1.0, 2.0, 1.0, 1.0, 1.0
    }; ///< Sigmas for [s pos, s vel, s acc, d pos, d vel, d acc] sampling
  } goalSampler;

  void Parse(const std::string& filename);
};

} // namespace pathplanning

#endif
