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

  double speedLimit = Mph2Mps(50.0);

  double timeStep = 0.02; ///< Seconds

  double timeHorizon = 2.0; ///< Time horizon for prediction

  int numPoints = 50.0;

  struct
  {
    double timeStep = 0.02;

  } simulator;

  struct
  {
    // Tracking radius
    double nonEgoSearchRadius = 30.0;

    // Deprecated
    int numMeasurementsToTrack = 30;
  } tracker;

  struct
  {
    // Use goal sampler or not
    bool use = true;

    // Number of samples to be generated
    int numSamplesPerTimeStep = 20;

    // Sample time step w.r.t. sample time
    double sampleTimeStep = 0.2;

    int numTimeSteps = 3;

    // Sigmas for [s pos, s vel, s acc, d pos, d vel, d acc] sampling
    std::array<double, 6> sampleSigmas = { 10.0, 1.0, 2.0, 1.0, 1.0, 1.0 };
  } goalSampler;

  struct
  {
    double planningDistance = 60.0;

    // For evaluation
    double maxJerk = 10.0;

    // For evaluation
    double maxAcc = 10.0;

    // For evaluation
    double expectedAccInOneSec = 2.0;

    // For evaluation
    double expectedJerkInOneSec = 1.0;

    // Collision checking radius in addition to vehicle bounding box
    double collisionCheckingRadius = 1.5;

    // Collision checking time step
    double collisionCheckingTimeStep = 0.01;

    // Max tim for generating feasible trajectory
    double maxTime = 5.0;

    // Time step for generating feasible trajectory
    double timeResolution = 0.02;

    // Sigmas for [s pos, s vel, s acc, d pos, d vel, d acc] trajectory
    // evaluation
    std::array<double, 6> evalSigmas = { 10.0, 1.0, 2.0, 1.0, 1.0, 1.0 };

    // Max curvature of the trajectory, for evaluation
    double maxCurvature = 30.0;

  } trajectory;

  // Current driver profile name to be used
  std::string driverProfileName;

  // Cost function weights to be used for trajectory generation, tweaking the
  // weight will result in different behaviors
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
    double totalAcc = 1.0;
    double maxAcc = 1.0;
    double totalJerk = 1.0;
    double maxJerk = 1.0;
  } driverProfile;

  void Parse(const std::string& filename);
};

} // namespace pathplanning

#endif
