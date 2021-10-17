#include "jmt.h"

#include "Eigen/Dense"

namespace pathplanning {

QuinticFunctor JMT::Solve1D(const std::array<double, 6> &params,
                            const double t) {
  std::array<double, 6> coeffs;
  Eigen::Matrix3d A;
  const double t2 = t * t;
  const double t3 = t2 * t;
  const double t4 = t3 * t;
  const double t5 = t4 * t;
  // clang-format off
  A <<
    t3     , t4      , t5      ,
    3 * t2 , 4 * t3  , 5 * t4  ,
    6 * t  , 12 * t2 , 20 * t3 ;
  // clang-format on

  Eigen::Vector3d b = {params[3], params[4], params[5]};

  // See https://eigen.tuxfamily.org/dox-devel/group__TutorialLinearAlgebra.html
  Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);
  coeffs[0] = params[0];
  coeffs[1] = params[1];
  coeffs[2] = params[2] / 2.0;
  coeffs[3] = x[0];
  coeffs[4] = x[1];
  coeffs[5] = x[2];
  return QuinticFunctor(coeffs);
}

SDFunctor JMT::Solve2D(const std::array<double, 6> &sParams,
                       const std::array<double, 6> &dParams, const double t) {
  return SDFunctor(JMT::Solve1D(sParams, t), JMT::Solve1D(dParams, t));
}

JMTTrajectory JMT::ComputeTrajectory(const std::array<double, 6> &sParams,
                                     const std::array<double, 6> &dParams,
                                     const double t) {
  return JMTTrajectory(JMT::Solve2D(sParams, dParams, t), t);
}

}  // namespace pathplanning
