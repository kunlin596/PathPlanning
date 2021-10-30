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
  if (j.count("sdHorizon")) {
    sdHorizon = j["sdHorizon"];
  }
  if (j.count("numPoints")) {
    numPoints = j["numPoints"];
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

  if (j.count("trajectoryEvaluation")) {
    const auto& trajectoryEvaluationJson = j["trajectoryEvaluation"];
    if (trajectoryEvaluationJson.count("maxJerk")) {
      trajectoryEvaluation.maxJerk = trajectoryEvaluationJson["maxJerk"];
    }
    if (trajectoryEvaluationJson.count("maxAcc")) {
      trajectoryEvaluation.maxAcc = trajectoryEvaluationJson["maxAcc"];
    }
    if (trajectoryEvaluationJson.count("expectedAccInOneSec")) {
      trajectoryEvaluation.expectedAccInOneSec =
        trajectoryEvaluationJson["expectedAccInOneSec"];
    }
    if (trajectoryEvaluationJson.count("expectedJerkInOneSec")) {
      trajectoryEvaluation.expectedJerkInOneSec =
        trajectoryEvaluationJson["expectedJerkInOneSec"];
    }
    if (trajectoryEvaluationJson.count("collisionCheckingRadius")) {
      trajectoryEvaluation.collisionCheckingRadius =
        trajectoryEvaluationJson["collisionCheckingRadius"];
    }
    if (trajectoryEvaluationJson.count("evalSigmas")) {
      trajectoryEvaluation.evalSigmas = trajectoryEvaluationJson["evalSigmas"];
    }
  }
  if (j.count("goalSampler")) {
    const auto& goalSamplerJson = j["goalSampler"];
    if (goalSamplerJson.count("use")) {
      goalSampler.use = goalSamplerJson["use"];
    }
    if (goalSamplerJson.count("numSamples")) {
      goalSampler.numSamples = goalSamplerJson["numSamples"];
    }
    if (goalSamplerJson.count("sampleTimeStep")) {
      goalSampler.sampleTimeStep = goalSamplerJson["sampleTimeStep"];
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
          driverProfile.exceedsSpeedLimit =
            driverProfileJson["exceedsSpeedLimit"];
        }
        if (driverProfileJson.count("efficiency")) {
          driverProfile.efficiency = driverProfileJson["efficiency"];
        }
        if (driverProfileJson.count("totalAccel")) {
          driverProfile.totalAccel = driverProfileJson["totalAccel"];
        }
        if (driverProfileJson.count("maxAccel")) {
          driverProfile.maxAccel = driverProfileJson["maxAccel"];
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
