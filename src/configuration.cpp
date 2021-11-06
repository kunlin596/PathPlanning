#include "configuration.h"

namespace pathplanning {
void
Configuration::Parse(const std::string& filename)
{
  using namespace nlohmann;
  json j;
  if (not filename.empty()) {
    std::ifstream ifs(filename);
    ifs >> j;
    std::cout << j << std::endl;
  }

  if (j.count("serverPort")) {
    serverPort = j["serverPort"];
  }
  if (j.count("speedLimit")) {
    speedLimit = j["speedLimit"];
  }
  if (j.count("timeStep")) {
    timeStep = j["timeStep"];
  }
  if (j.count("timeHorizon")) {
    timeHorizon = j["timeHorizon"];
  }

  if (j.count("tracker")) {
    const auto& trackerJson = j["tracker"];
    if (trackerJson.count("numMeasurementsToTrack")) {
      tracker.numMeasurementsToTrack = trackerJson["numMeasurementsToTrack"];
    }
    if (trackerJson.count("nonEgoSearchRadius")) {
      tracker.nonEgoSearchRadius = trackerJson["nonEgoSearchRadius"];
    }
  }

  if (j.count("trajectory")) {
    const auto& trajectoryEvaluationJson = j["trajectory"];
    if (trajectoryEvaluationJson.count("maxTime")) {
      trajectory.maxTime = trajectoryEvaluationJson["maxTime"];
    }
    if (trajectoryEvaluationJson.count("maxCurvature")) {
      trajectory.maxCurvature = trajectoryEvaluationJson["maxCurvature"];
    }
    if (trajectoryEvaluationJson.count("timeResolution")) {
      trajectory.timeResolution = trajectoryEvaluationJson["timeResolution"];
    }
    if (trajectoryEvaluationJson.count("maxJerk")) {
      trajectory.maxJerk = trajectoryEvaluationJson["maxJerk"];
    }
    if (trajectoryEvaluationJson.count("maxAcc")) {
      trajectory.maxAcc = trajectoryEvaluationJson["maxAcc"];
    }
    if (trajectoryEvaluationJson.count("expectedAccInOneSec")) {
      trajectory.expectedAccInOneSec = trajectoryEvaluationJson["expectedAccInOneSec"];
    }
    if (trajectoryEvaluationJson.count("expectedJerkInOneSec")) {
      trajectory.expectedJerkInOneSec = trajectoryEvaluationJson["expectedJerkInOneSec"];
    }
    if (trajectoryEvaluationJson.count("collisionCheckingRadius")) {
      trajectory.collisionCheckingRadius = trajectoryEvaluationJson["collisionCheckingRadius"];
    }
    if (trajectoryEvaluationJson.count("collisionCheckingTimeStep")) {
      trajectory.collisionCheckingTimeStep = trajectoryEvaluationJson["collisionCheckingTimeStep"];
    }
    if (trajectoryEvaluationJson.count("evalSigmas")) {
      trajectory.evalSigmas = trajectoryEvaluationJson["evalSigmas"];
    }
  }
  if (j.count("goalSampler")) {
    const auto& goalSamplerJson = j["goalSampler"];
    if (goalSamplerJson.count("use")) {
      goalSampler.use = goalSamplerJson["use"];
    }
    if (goalSamplerJson.count("numSamplesPerTimeStep")) {
      goalSampler.numSamplesPerTimeStep = goalSamplerJson["numSamplesPerTimeStep"];
    }
    if (goalSamplerJson.count("sampleTimeStep")) {
      goalSampler.sampleTimeStep = goalSamplerJson["sampleTimeStep"];
    }
    if (goalSamplerJson.count("numTimeSteps")) {
      goalSampler.numTimeSteps = goalSamplerJson["numTimeSteps"];
    }
    if (goalSamplerJson.count("sampleSigmas")) {
      goalSampler.sampleSigmas = goalSamplerJson["sampleSigmas"];
    }
  }
  if (j.count("driverProfileName")) {
    driverProfileName = j["driverProfileName"];
  }

  if (!driverProfileName.empty()) {
    if (j.count("driverProfiles")) {
      const auto& driverProfilesJson = j["driverProfiles"];
      if (driverProfilesJson.count(driverProfileName)) {
        const auto& driverProfileJson = driverProfilesJson[driverProfileName];
        if (driverProfileJson.count("timeDiff")) {
          driverProfile.timeDiff = driverProfileJson["timeDiff"];
        }
        if (driverProfileJson.count("sDiff")) {
          driverProfile.sDiff = driverProfileJson["sDiff"];
        }
        if (driverProfileJson.count("dDiff")) {
          driverProfile.dDiff = driverProfileJson["dDiff"];
        }
        if (driverProfileJson.count("collision")) {
          driverProfile.collision = driverProfileJson["collision"];
        }
        if (driverProfileJson.count("buffer")) {
          driverProfile.buffer = driverProfileJson["buffer"];
        }
        if (driverProfileJson.count("staysOnRoad")) {
          driverProfile.staysOnRoad = driverProfileJson["staysOnRoad"];
        }
        if (driverProfileJson.count("exceedsSpeedLimit")) {
          driverProfile.exceedsSpeedLimit = driverProfileJson["exceedsSpeedLimit"];
        }
        if (driverProfileJson.count("efficiency")) {
          driverProfile.efficiency = driverProfileJson["efficiency"];
        }
        if (driverProfileJson.count("totalAcc")) {
          driverProfile.totalAcc = driverProfileJson["totalAcc"];
        }
        if (driverProfileJson.count("maxAcc")) {
          driverProfile.maxAcc = driverProfileJson["maxAcc"];
        }
        if (driverProfileJson.count("totalJerk")) {
          driverProfile.totalJerk = driverProfileJson["totalJerk"];
        }
        if (driverProfileJson.count("maxJerk")) {
          driverProfile.maxJerk = driverProfileJson["maxJerk"];
        }
      }
    }
  }
}
} // namespace pathplannin
