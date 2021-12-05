#include "../configuration.h"
#include "../jmt.h"
#include "../log.h"

#include <gtest/gtest.h>

namespace {
using namespace pathplanning;
}

TEST(JMTTest, Solve1d_Basic)
{
  JMTTrajectory1d traj;
  Vector6d answer;
  Vector6d conditions;

  conditions << 0.0, 10.0, 0.0, 10.0, 10.0, 0.0;
  traj = JMT::Solve1d_6DoF(conditions, 1.0);
  answer << 0.0, 10.0, 0.0, 0.0, 0.0, 0.0;

  EXPECT_NEAR((traj.GetPositionFn().coeffs - answer).norm(), 0.0, 1e-8);

  conditions << 0.0, 10.0, 0.0, 20.0, 15.0, 20.0;
  traj = JMT::Solve1d_6DoF(conditions, 2.0);
  answer << 0.0, 10.0, 0.0, 0.0, -0.625, 0.3125;
  EXPECT_NEAR((traj.GetPositionFn().coeffs - answer).norm(), 0.0, 1e-8);

  conditions << 5.0, 10.0, 2.0, -30.0, -20.0, -4.0;
  traj = JMT::Solve1d_6DoF(conditions, 5.0);
  answer << 5.0, 10.0, 1.0, -3.0, 0.64, -0.0432;
  EXPECT_NEAR((traj.GetPositionFn().coeffs - answer).norm(), 0.0, 1e-8);
}
