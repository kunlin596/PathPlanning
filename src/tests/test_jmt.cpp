#include "../configuration.h"
#include "../jmt.h"
#include "../log.h"

#include <gtest/gtest.h>

namespace {
using namespace pathplanning;
}

TEST(JMTTest, Solve1d_ConstantSpeedTest)
{
  double vel = 30.0;
  double time = 10.0;
  double pos = vel * time;
  double dt = 0.02;

  Vector6d conditions;
  conditions << 0.0, vel, 0.0, pos, vel, 0.0;

  JMTTrajectory1d traj1d = JMT::Solve1d(conditions, time);

  EXPECT_NEAR(traj1d(time)[0], pos, 1e-8);
  double currTime = 0.0;
  while (currTime < time + 1e-6) {
    Vector6d kinematic = traj1d(currTime);
    EXPECT_NEAR(kinematic[1], vel, 1e-8);
    EXPECT_NEAR(kinematic[2], 0.0, 1e-8);
    currTime += dt;
  }
}

TEST(JMTTest, Solve1d_EndConditionTest)
{
  double vel = 30.0;
  double acc = 5.0;
  double time = 20.0;
  double pos = vel * time;

  Configuration conf;
  Vector6d conditions;
  JMTTrajectory1d traj1d;

  GaussianSampler1D<> posSampler(pos, 10.0);
  GaussianSampler1D<> velSampler(vel, 10.0);
  GaussianSampler1D<> accSampler(acc, 10.0);

  for (int i = 0; i < 50; ++i) {
    conditions << 0.0, vel, 0.0, posSampler.Sample(), velSampler.Sample(), accSampler.Sample();
    traj1d = JMT::Solve1d(conditions, time);
    Vector6d kinematics = traj1d(time);
    EXPECT_NEAR(kinematics[0], conditions[3], 1e-8);
    EXPECT_NEAR(kinematics[1], conditions[4], 1e-8);
    EXPECT_NEAR(kinematics[2], conditions[5], 1e-8);
  }
}

TEST(JMTTest, Solve2d_EndConditionTest)
{
  double vel = 30.0;
  double acc = 5.0;
  double time = 20.0;
  double pos = vel * time;

  Configuration conf;
  Matrix62d conditions;
  JMTTrajectory2d traj2d;

  GaussianSampler1D<> posSampler(pos, 10.0);
  GaussianSampler1D<> velSampler(vel, 10.0);
  GaussianSampler1D<> accSampler(acc, 10.0);

  for (int i = 0; i < 50; ++i) {
    conditions.col(0) << 0.0, vel, 0.0, posSampler.Sample(), velSampler.Sample(), accSampler.Sample();
    conditions.col(1) << 0.0, vel, acc, posSampler.Sample(), velSampler.Sample(), accSampler.Sample();
    traj2d = JMT::Solve2d(conditions, time);
    Matrix62d kinematics = traj2d(time);
    // s
    EXPECT_NEAR(kinematics(0, 0), conditions(3, 0), 1e-8);
    EXPECT_NEAR(kinematics(1, 0), conditions(4, 0), 1e-8);
    EXPECT_NEAR(kinematics(2, 0), conditions(5, 0), 1e-8);
    // d
    EXPECT_NEAR(kinematics(0, 1), conditions(3, 1), 1e-8);
    EXPECT_NEAR(kinematics(1, 1), conditions(4, 1), 1e-8);
    EXPECT_NEAR(kinematics(2, 1), conditions(5, 1), 1e-8);
  }
}
