#include <uWS/uWS.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "behaviorplanner.h"
#include "pathplanner.h"
#include "json.hpp"

int main() {
  using nlohmann::json;
  using std::string;
  using std::vector;
  using namespace pathplanning;

  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  NaviMap naviMap;

  // Waypoint map to read from
  const string mapFile = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  const double max_s = 6945.554;

  std::ifstream in(mapFile.c_str(), std::ifstream::in);

  // Deserialize map data from file
  string line;
  while (getline(in, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float dx;
    float dy;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> dx;
    iss >> dy;
    naviMap.x.push_back(x);
    naviMap.y.push_back(y);
    naviMap.s.push_back(s);
    naviMap.dx.push_back(dx);
    naviMap.dy.push_back(dy);
  }

  naviMap.road.lanes = {Lane(0), Lane(1), Lane(2)};

  // Lane reference for navigation, initial value is the initial lane,
  // will be updated over time.
  // In this project, the land id can be 0, 1, 2 (3 lanes).
  // Speed reference for navidation, initial value is the initial speed,
  // will be updated over time, unit is in mile (simulator is using this unit, Orz).
  Behavior currBehavior(naviMap.road.lanes[1], 0.0, BehaviorState::kLaneKeeping);

  BehaviorPlanner behaviorPlanner(naviMap);
  GoalGenerator goalGenerator(naviMap);
  PathPlanner pathPlanner(naviMap);

  // Main event loop callback when we receive something from the simulator.
  // These parameters are specific to uWS communication.
  h.onMessage(
    [&naviMap, &currBehavior, &behaviorPlanner, &goalGenerator, &pathPlanner] (
      uWS::WebSocket<uWS::SERVER> ws,
      char *data,
      size_t length,
      uWS::OpCode opCode
    ) {

    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        std::string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          //
          // Main car's localization Data
          //

          CarState currCarState(
            {deg2rad(j[1]["yaw"]), j[1]["x"], j[1]["y"]},
            {j[1]["s"], j[1]["d"]},
            j[1]["speed"]
          );

          cout << currCarState << endl;
 
          // Previous path data given to the Planner
          Path prevPath = ConvertXYToPath(j[1]["previous_path_x"], j[1]["previous_path_y"]);
          // cout << prevPath << endl;

          pathPlanner.UpdatePathCache(prevPath);

          // Previous path's end s and d values
          const std::array<double, 2> endPathFrenetPose = { j[1]["end_path_s"], j[1]["end_path_d"] };

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          SensorFusions sensorFusions = SensorFusion::Read(j[1]["sensor_fusion"]);

          pathPlanner.UpdateCarState(currCarState);

          //
          // Main path generation loop
          //

          // Group sensor fusion results per lane;
          std::vector<SensorFusions> laneSensorFusions(3);
          for (const SensorFusion &sensorFusion : sensorFusions) {
            const int laneId = naviMap.road.GetLandId(sensorFusion.frenetPose[1]);
            if (laneId >= 0) {
              laneSensorFusions[laneId].push_back(sensorFusion);
            }
          }

          std::array<double, 3> averSpeeds = { 0.0, 0.0, 0.0 };
          for (size_t i = 0; i < averSpeeds.size(); ++i) {
            if (laneSensorFusions[i].size() < 1) {
              continue;
            }

            for (const auto &sensorFusion : laneSensorFusions[i]) {
              averSpeeds[i] += sensorFusion.speed;
            }
          }

          for (size_t i = 0; i < averSpeeds.size(); ++i) {
            averSpeeds[i] /= static_cast<double>(laneSensorFusions[i].size());
          }

          cout << "Lane averSpeeds=" << averSpeeds << endl;

          std::vector<BehaviorState> possibleStates
            = behaviorPlanner.GetSuccessorStates(currBehavior.state, currBehavior.lane.id);

          std::vector<std::tuple<BehaviorState, Goal, Path, double>> pathCache;
          for (const auto &candidateBehaviorState : possibleStates) {
            Goal goal = goalGenerator.GenerateGoal(
              candidateBehaviorState, currBehavior.targetSpeed, sensorFusions, currCarState, prevPath);

            Path candidatePath = pathPlanner.GeneratePath(goal, currBehavior);
            double score = pathPlanner.EvaluatePath(
              goal, candidatePath, laneSensorFusions, averSpeeds);

            pathCache.push_back({
              candidateBehaviorState, goal, candidatePath, score
            });
          }

          decltype(pathCache)::iterator bestPathItr = std::min_element(
            pathCache.begin(), pathCache.end(),
            [] (
              const std::tuple<BehaviorState, Goal, Path, double> &e1,
              const std::tuple<BehaviorState, Goal, Path, double> &e2)
            {
              return std::get<3>(e1) > std::get<3>(e2);
            });

          size_t bestIndex = std::distance(pathCache.begin(), bestPathItr);

          // Update behavior state with new data
          currBehavior.state = std::get<0>(pathCache[bestIndex]);
          Goal bestGoal = std::get<1>(pathCache[bestIndex]);
          cout << "Best goal, " << bestGoal << endl;
          currBehavior.targetSpeed = bestGoal.speed;
          Path newPath = std::get<2>(pathCache[bestIndex]);

          json msgJson;

          std::vector<double> nextXValues, nextYValues;
          std::tie(nextXValues, nextYValues) = ConverPathToXY(newPath);
          msgJson["next_x"] = nextXValues;
          msgJson["next_y"] = nextYValues;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  });  // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](
    uWS::WebSocket<uWS::SERVER> ws, int code,
    char *message, size_t length
  ) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}
