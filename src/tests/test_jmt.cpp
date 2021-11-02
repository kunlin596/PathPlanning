#include "../jmt.h"
// #include "json.hpp"
#include "../configuration.h"
#include "../log.h"

#include <gtest/gtest.h>

namespace {
using namespace pathplanning;
}

TEST(JMTTest, Solve1DConstantSpeedTest)
{
  double vel = 30.0;
  double time = 10.0;
  double s = vel * time;
  double dt = 0.02;

  JMTTrajectory1D traj1d =
    JMT::Solve1D({ 0.0, vel, 0.0 }, { s, vel, 0.0 }, time);

  EXPECT_NEAR(traj1d(time)[0], s, 1e-8);
  double currTime = 0.0;
  while (currTime < time + 1e-6) {
    auto kinematic = traj1d(currTime);
    EXPECT_NEAR(kinematic[1], vel, 1e-8);
    EXPECT_NEAR(kinematic[2], 0.0, 1e-8);
    currTime += dt;
  }
}

TEST(JMTTest, Solve1DEndKinematicTest)
{
  double vel = 30.0;
  double accel = 5.0;
  double time = 10.0;
  double s = vel * time;
  double dt = 0.02;

  JMTTrajectory1D traj1d =
    JMT::Solve1D({ 0.0, 0.0, 0.0 }, { s, vel, accel }, time);

  EXPECT_NEAR(traj1d(time)[0], s, 1e-8);
  EXPECT_NEAR(traj1d(time)[1], vel, 1e-8);
  EXPECT_NEAR(traj1d(time)[2], accel, 1e-8);

  traj1d = JMT::Solve1D({ 0.0, vel * 2, accel / 2.0 }, { s, vel, accel }, time);
  EXPECT_NEAR(traj1d(time)[0], s, 1e-8);
  EXPECT_NEAR(traj1d(time)[1], vel, 1e-8);
  EXPECT_NEAR(traj1d(time)[2], accel, 1e-8);

  traj1d =
    JMT::Solve1D({ 0.0, vel * 2, accel / 2.0 }, { s, vel, -accel }, time);
  EXPECT_NEAR(traj1d(time)[0], s, 1e-8);
  EXPECT_NEAR(traj1d(time)[1], vel, 1e-8);
  EXPECT_NEAR(traj1d(time)[2], -accel, 1e-8);
}

TEST(JMTTest, ValidationTest)
{
  double vel = 30.0;
  double accel = 5.0;
  double time = 1.0;
  double s = vel * time;
  double dt = 0.02;

  Configuration conf;

  JMTTrajectory1D traj1d =
    JMT::Solve1D({ 0.0, 0.0, 0.0 }, { s, vel, accel }, time);

  EXPECT_NEAR(traj1d(time)[0], s, 1e-8);
  EXPECT_NEAR(traj1d(time)[1], vel, 1e-8);
  EXPECT_NEAR(traj1d(time)[2], accel, 1e-8);
  EXPECT_FALSE(traj1d.IsValid(conf));

  traj1d = JMT::Solve1D({ 0.0, vel, accel / 2.0 }, { s, vel, -accel }, time);
  EXPECT_NEAR(traj1d(time)[0], s, 1e-8);
  EXPECT_NEAR(traj1d(time)[1], vel, 1e-8);
  EXPECT_NEAR(traj1d(time)[2], -accel, 1e-8);
  EXPECT_TRUE(traj1d.IsValid(conf));

  traj1d = JMT::Solve1D({ 0.0, 0.0, 0.0 }, { s, vel / 2.0, accel / 2.0 }, time);

  EXPECT_NEAR(traj1d(time)[0], s, 1e-8);
  EXPECT_NEAR(traj1d(time)[1], vel / 2.0, 1e-8);
  EXPECT_NEAR(traj1d(time)[2], accel / 2.0, 1e-8);
  EXPECT_FALSE(traj1d.IsValid(conf));
}
