#pragma once

#include "Eigen-3.3/Eigen/Dense"
#include "math.h"
#include <memory>

namespace Eigen {
using Vector6d = Matrix<double, 6, 1>;
using Matrix6d = Matrix<double, 6, 6>;
using Matrix62d = Matrix<double, 6, 2>;
using Matrix32d = Matrix<double, 3, 2>;
};

namespace pathplanning {

using Eigen::Matrix32d;
using Eigen::Matrix62d;
using Eigen::Matrix6d;
using Eigen::Vector6d;

/**
 * @brief      This class describes a goal sampler.
 *
 * Goal sampler is trying to find the best goal w.r.t. target nominal configuration
 */
class GoalSampler
{
public:
  explicit GoalSampler(const Matrix32d& nominalConf, const std::array<double, 6>& sigmas);

  Matrix32d Sample() const;

private:
  std::array<std::unique_ptr<GaussianSampler1D<>>, 6> _samplers;
};

} // namespace pathplanning
