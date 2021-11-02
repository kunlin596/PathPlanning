// #include "json.hpp"
#include "../log.h"
#include "../traj_evaluator.h"

#include <gtest/gtest.h>

namespace {
using namespace pathplanning;
}

TEST(TrajEvaluator, TimeDiffTest)
{
  VehicleConfiguration startConf(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  VehicleConfiguration goalConf(30.0, 20.0, 1.0, 0.0, 1.0, 1.0);
  double time = 1.5;
  JMTTrajectory2D traj = JMT::ComputeTrajectory(startConf, goalConf, time);

  TrackedVehicleMap vehicles;
  vehicles[0] = Vehicle(0, VehicleConfiguration(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

  costs::TimeDiffCost costFunc;
  JMTTrajectoryEvaluator::Options options;

  double cost = costFunc.Compute(traj, goalConf, time, vehicles, options);
  EXPECT_EQ(cost, 0.0);

  double maxCost = Gaussian1D(time, 1.0, time);

  cost = costFunc.Compute(
    traj, goalConf, std::numeric_limits<double>::max(), vehicles, options);
  EXPECT_NEAR(cost, maxCost, 1e-6);

  cost = costFunc.Compute(
    traj, goalConf, -std::numeric_limits<double>::max(), vehicles, options);
  EXPECT_NEAR(cost, maxCost, 1e-6);
}

TEST(TrajEvaluator, SDiffTest)
{
  VehicleConfiguration startConf(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  VehicleConfiguration goalConf(10.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  double time = 1.5;
  JMTTrajectory2D traj = JMT::ComputeTrajectory(startConf, goalConf, time);

  TrackedVehicleMap vehicles;
  vehicles[0] = Vehicle(0, VehicleConfiguration(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

  costs::SDiffCost costFunc;
  JMTTrajectoryEvaluator::Options options;

  double cost = costFunc.Compute(traj, goalConf, time, vehicles, options);
  EXPECT_EQ(cost, 0.0);
}

TEST(TrajEvaluator, CollisionTest)
{
  VehicleConfiguration startConf(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  VehicleConfiguration goalConf(30.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  double time = 1.5;
  JMTTrajectory2D traj = JMT::ComputeTrajectory(startConf, goalConf, time);

  TrackedVehicleMap vehicles;
  vehicles[0] = Vehicle(0, VehicleConfiguration(20.0, 1.0, 0.0, 0.0, 0.0, 0.0));
}
