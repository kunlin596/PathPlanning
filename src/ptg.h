#ifndef PATHPLANNING_PTG_H
#define PATHPLANNING_PTG_H

#include <memory>

#include "jmt.h"
#include "math.h"
#include "path.h"
#include "vehicle.h"

namespace pathplanning {

class GoalSampler {
 public:
  explicit GoalSampler(const double s, const double sSigma, const double d,
                       const double dSigma);

  std::array<double, 2> Sample() const;

 private:
  // Make it constant for this project
  std::unique_ptr<GaussianSampler1D<>> _sSampler;
  std::unique_ptr<GaussianSampler1D<>> _dSampler;
};

class PolynomialTrajectoryGenerator {
 public:
  struct Options {
    uint32_t numSamples = 10;
    double timeStep = 0.02;
    double sSamplerSigma = 0.5;
    double dSamplerSigma = 0.1;
  };

  Waypoints Generate(const VehicleKinParams &startParams,
                     const VehicleKinParams &end,
                     const std::vector<Vehicle> &predictions,
                     const double t) const;

 private:
  Options _options;
  JMTTrajectoryValidator _validator;
};

};  // namespace pathplanning

#endif
