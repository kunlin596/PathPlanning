#include "jmt.h"
// #include "json.hpp"
#include "log.h"

#include <gtest/gtest.h>

namespace {
using namespace pathplanning;
}

// Demonstrate some basic assertions.
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

  // using namespace nlohmann;
  // json j;

  JMTTrajectory1D::ValidationParams params({ 0.0, 50.0 }, { -10.0, 10.0 });

  JMTTrajectory1D traj1d =
    JMT::Solve1D({ 0.0, 0.0, 0.0 }, { s, vel, accel }, time);
  // j["traj1d_1"] = traj1d.Dump();

  EXPECT_NEAR(traj1d(time)[0], s, 1e-8);
  EXPECT_NEAR(traj1d(time)[1], vel, 1e-8);
  EXPECT_NEAR(traj1d(time)[2], accel, 1e-8);
  EXPECT_FALSE(traj1d.IsValid(params));

  traj1d = JMT::Solve1D({ 0.0, vel, accel / 2.0 }, { s, vel, -accel }, time);
  EXPECT_NEAR(traj1d(time)[0], s, 1e-8);
  EXPECT_NEAR(traj1d(time)[1], vel, 1e-8);
  EXPECT_NEAR(traj1d(time)[2], -accel, 1e-8);
  EXPECT_TRUE(traj1d.IsValid(params));
  // j["traj1d_2"] = traj1d.Dump();

  traj1d = JMT::Solve1D({ 0.0, 0.0, 0.0 }, { s, vel / 2.0, accel / 2.0 }, time);

  EXPECT_NEAR(traj1d(time)[0], s, 1e-8);
  EXPECT_NEAR(traj1d(time)[1], vel / 2.0, 1e-8);
  EXPECT_NEAR(traj1d(time)[2], accel / 2.0, 1e-8);
  EXPECT_FALSE(traj1d.IsValid(params));
  // j["traj1d_3"] = traj1d.Dump();
  // std::ofstream o(fmt::format("traj1d_time_{:0.3f}.json", time));
  // o << std::setw(4) << j << std::endl;
}

// TEST(JMTTest, Solve2DTest)
// {
//   double currTime = 0.5;
//   while (currTime < 10.0 + 1e-6) {
//     JMTTrajectory2D traj2d = JMT::Solve2D(
//       { 0.0, 0.0, 0.0, 30.0, 20.0, 1.0 }, { 0.0, 0.0, 0.0, 0.0, 10, 2.0 }, currTime);
//     currTime += 0.5;
//     auto j = traj2d.Dump();
//     std::ofstream o(fmt::format("traj2d-{:.3f}.json", currTime));
//     o << std::setw(4) << j << std::endl;
//   }
// }

// TEST(JMTTest, TimeTest)
// {
//   using namespace nlohmann;
//   json j;

//   double time = 1.0;
//   do {
//     JMTTrajectory1D traj1d =
//       JMT::Solve1D({ 0.0, 0.0, 0.0 }, { 30.0, 10.0, 1.0 }, time);
//     j[fmt::format("traj1d_time_exp,{:.3f}", time)] = traj1d.Dump();
//     time += 0.1;
//   } while (time < 4.0);
//   std::ofstream o("traj1d_time_exp.json");
//   o << std::setw(4) << j << std::endl;
// }

// TEST(JMTTest, MergeTest)
// {
//   std::vector<std::array<double, 3>> points = { { 0.0, 0.0, 0.0 },
//                                                 { 30.0, 10.0, 2.0 },
//                                                 { 60.0, 20.0, 1.0 },
//                                                 { 70.0, 40.0, 2.0 },
//                                                 { 100.0, 49.0, 2.0 } };
//   std::vector<double> times = { 2.0, 2.0, 2.0, 1.0 };
//   using namespace nlohmann;
//   json j;

//   std::vector<JMTTrajectory1D> trajs;
//   for (int i = 1; i < points.size(); ++i) {
//     double t = times[i - 1];
//     auto start = points[i - 1];
//     auto end = points[i];
//     JMTTrajectory1D traj1d = JMT::Solve1D(start, end, t);
//     j[fmt::format("traj1d_seg_{:d}", i)] = traj1d.Dump();
//   }
//   std::ofstream o("traj1d_merge_test.json");
//   o << std::setw(4) << j << std::endl;
// }
