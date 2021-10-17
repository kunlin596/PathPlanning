#ifndef PATHPLANNING_PTG_H
#define PATHPLANNING_PTG_H

#include "path.h"

namespace pathplanning {

class PolynomialTrajectoryGenerator {
 public:
  struct Options {
    uint32_t numSamples = 10;
    double timeStep = 0.02;
    double sSamplerSigma = 0.5;
    double dSamplerSigma = 0.1;
  };

  Waypoints Generate(const std::array<double, 6> &sParams,
                     const std::array<double, 6> &dParams,
                     const double t) const;

 private:
  Options _options;
};

};  // namespace pathplanning

#endif
