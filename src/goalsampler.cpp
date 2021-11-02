#include "goalsampler.h"

namespace pathplanning {

GoalSampler::GoalSampler(const Matrix32d& nominalConf, const std::array<double, 6>& sigmas)
{
  // s
  _samplers[0] = std::make_unique<GaussianSampler1D<>>(nominalConf(0, 0), sigmas[0]);
  _samplers[1] = std::make_unique<GaussianSampler1D<>>(nominalConf(1, 0), sigmas[1]);
  _samplers[2] = std::make_unique<GaussianSampler1D<>>(nominalConf(2, 0), sigmas[2]);
  // d
  _samplers[3] = std::make_unique<GaussianSampler1D<>>(nominalConf(0, 1), sigmas[3]);
  _samplers[4] = std::make_unique<GaussianSampler1D<>>(nominalConf(1, 1), sigmas[4]);
  _samplers[5] = std::make_unique<GaussianSampler1D<>>(nominalConf(2, 1), sigmas[5]);
}

Matrix32d
GoalSampler::Sample() const
{
  Matrix32d sample;
  sample.col(0) << _samplers[0]->Sample(), _samplers[1]->Sample(), _samplers[2]->Sample();
  sample.col(1) << _samplers[3]->Sample(), _samplers[4]->Sample(), _samplers[5]->Sample();
  return sample;
}

} // namespace pathplanning
