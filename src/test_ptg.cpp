#include <cmath>

#include "json.hpp"
#include "log.h"
#include "ptg.h"

namespace {
using namespace pathplanning;

void
Test(PolynomialTrajectoryGenerator& g,
     const Map::ConstPtr& pMap,
     const VehicleConfiguration& start,
     const VehicleConfiguration& goal)
{
  TrackedVehicleMap trackedVehicleMap;
  Waypoints prevPath;

  Vehicle startState(0, start, 30, 0.02);
  Vehicle goalState(0, goal, 30, 0.02);

  SPDLOG_INFO(start);
  SPDLOG_INFO(goal);
  double targetExecutionTime = 10.0;

  Waypoints xywaypoints;
  JMTTrajectory trajectory;
  std::tie(xywaypoints, trajectory) = g.GeneratePath(
    startState, goalState, trackedVehicleMap, targetExecutionTime);

  auto computed = trajectory(targetExecutionTime);

  double timestep = 0.02;
  int count = int(targetExecutionTime / timestep);

  for (int i = 0; i < count + 1; ++i) {
    auto computed = trajectory(i * timestep);
    SPDLOG_INFO("computed={}", computed);
  }

  auto startXY = pMap->GetXY(start.sPos, start.dPos);
  auto goalXY = pMap->GetXY(goal.sPos, goal.dPos);
  auto computedXY = pMap->GetXY(computed.sPos, computed.dPos);

  SPDLOG_INFO(
    "startXY={}, goalXY={}, computedXY={}", startXY, goalXY, computedXY);

  double diffX = goalXY[0] - startXY[0];
  double diffY = goalXY[1] - startXY[1];
  SPDLOG_INFO("diffX={}, diffY={}", diffX, diffY);

  double xError = std::abs(xywaypoints[0][0] + diffX -
                           xywaypoints[xywaypoints.size() - 1][0]);
  double yError = std::abs(xywaypoints[0][1] + diffY -
                           xywaypoints[xywaypoints.size() - 1][1]);

  SPDLOG_INFO(
    "start{}, goal={}", xywaypoints[0], xywaypoints[xywaypoints.size() - 1]);
  SPDLOG_INFO("xError={}, yError={}", xError, yError);

  assert(xError < 0.8);
  assert(yError < 0.8);
}
} // namespace

int
main(int argc, char** argv)
{
  using namespace pathplanning;
  using nlohmann::json;

  Map::ConstPtr pMap = Map::CreateMap("../data/highway_map.csv");

  PolynomialTrajectoryGenerator::Options options;
  PolynomialTrajectoryGenerator g(pMap, options);

  auto record = pMap->Get(0);
  double offset = 30.0;

  // Move the car from the starting point of the map 30 meters ahead.
  VehicleConfiguration start;
  VehicleConfiguration goal;

  SPDLOG_INFO(" --- Moving forward ---");
  start = VehicleConfiguration(record[2], 0.0, 0.0, 0.0, 0.0, 0.0);
  goal = VehicleConfiguration(record[2] + offset, 0.0, 0.0, 0.0, 0.0, 0.0);
  Test(g, pMap, start, goal);

  SPDLOG_INFO(" --- Change lane and forward ---");
  start = VehicleConfiguration(record[2], 0.0, 0.0, 0.0, 0.0, 0.0);
  goal = VehicleConfiguration(record[2] + offset, 0.0, 0.0, 4.0, 0.0, 0.0);
  Test(g, pMap, start, goal);

  // The bigger offset, the higher drift in d will be!
  // error in y when,
  // - offset = 30: 0.62
  // - offset = 50: 0.76
  return 0;
}
