// #include "json.hpp"
#include "../configuration.h"
#include "../log.h"
#include "../traj_evaluator.h"

#include <gtest/gtest.h>

namespace {
using namespace pathplanning;
}

TEST(TrajEvaluator, TimeDiffTest)
{
  Configuration conf;
  VehicleConfiguration startConf(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  VehicleConfiguration goalConf(30.0, 20.0, 1.0, 0.0, 1.0, 1.0);
  double time = 1.5;
  JMTTrajectory2d traj = JMT::ComputeTrajectory(startConf, goalConf, time);

  TrackedVehicleMap vehicles;
  vehicles[0] = Vehicle(0, VehicleConfiguration(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

  costs::TimeDiffCost costFunc;

  double cost = costFunc.Compute(traj, goalConf, time, vehicles, conf);
  EXPECT_EQ(cost, 0.0);

  double maxCost = Gaussian1D(time, 1.0, time);

  cost = costFunc.Compute(
    traj, goalConf, std::numeric_limits<double>::max(), vehicles, conf);
  EXPECT_NEAR(cost, maxCost, 1e-6);

  cost = costFunc.Compute(
    traj, goalConf, -std::numeric_limits<double>::max(), vehicles, conf);
  EXPECT_NEAR(cost, maxCost, 1e-6);
}

TEST(TrajEvaluator, SDiffTest)
{
  Configuration conf;
  VehicleConfiguration startConf(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  VehicleConfiguration goalConf(10.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  double time = 1.5;
  JMTTrajectory2d traj = JMT::ComputeTrajectory(startConf, goalConf, time);

  TrackedVehicleMap vehicles;
  vehicles[0] = Vehicle(0, VehicleConfiguration(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

  costs::SDiffCost costFunc;

  double cost = costFunc.Compute(traj, goalConf, time, vehicles, conf);
  EXPECT_EQ(cost, 0.0);
}

TEST(TrajEvaluator, StaticCollisionTest)
{
  Configuration conf;
  VehicleConfiguration startConf(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  VehicleConfiguration goalConf(20.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  double time = 2.0;
  TrackedVehicleMap vehicles;
  costs::CollisionCost costFunc;

  JMTTrajectory2d traj = JMT::ComputeTrajectory(startConf, goalConf, time);

  vehicles[0] =
    Vehicle(0,
            VehicleConfiguration(
              10.0, 0.0, 0.0, VehicleConfiguration::Size - 1e-6, 0.0, 0.0));

  double cost = costFunc.Compute(traj, goalConf, time, vehicles, conf);
  EXPECT_EQ(cost, 1.0);

  vehicles[0] = Vehicle(
    0, VehicleConfiguration(10.0, 0.0, 0.0, VehicleConfiguration::Size + 1e-6, 0.0, 0.0));
  cost = costFunc.Compute(traj, goalConf, time, vehicles, conf);
  EXPECT_EQ(cost, 0.0);
}

TEST(TrajEvaluator, DynamicCollisionTest)
{
  Configuration conf;
  VehicleConfiguration startConf(0.0, 20.0, 0.0, 0.0, 0.0, 0.0);
  VehicleConfiguration goalConf(20.0, 30.0, 0.0, 0.0, 0.0, 0.0);
  double time = 2.0;
  JMTTrajectory2d traj = JMT::ComputeTrajectory(startConf, goalConf, time);

  TrackedVehicleMap vehicles;
  vehicles[0] =
    Vehicle(0, VehicleConfiguration(10.0, 1.0, 0.0, 0.0, 0.0, 0.0));

  costs::CollisionCost costFunc;

  double cost = costFunc.Compute(traj, goalConf, time, vehicles, conf);
  EXPECT_EQ(cost, 1.0);

  vehicles[0] =
    Vehicle(0, VehicleConfiguration(10.0, 5.0, 0.0, 0.0, 0.0, 0.0));

  cost = costFunc.Compute(traj, goalConf, time, vehicles, conf);
  EXPECT_EQ(cost, 1.0);
}
