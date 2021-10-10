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
  BehaviorPlanner behaviorPlanner;

  // Main event loop callback when we receive something from the simulator.
  // These parameters are specific to uWS communication.
  h.onMessage(
    [&naviMap, &currBehavior, &behaviorPlanner] (
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
          // BOOST_LOG_TRIVIAL(info) << currCarState;
 
          // Previous path data given to the Planner
          Path prevPath = ConvertXYToPath(
            j[1]["previous_path_x"],
            j[1]["previous_path_y"]
          );

          const size_t prevPathSize = prevPath.size();

          // Previous path's end s and d values
          std::array<double, 2> endPathFrenetPose = {
            j[1]["end_path_s"],
            j[1]["end_path_d"]
          };

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          SensorFusions sensorFusions = SensorFusion::Read(j[1]["sensor_fusion"]);

          std::vector<BehaviorState> possibleStates
            = behaviorPlanner.GetSuccessorStates(currBehavior.state, currBehavior.lane.id);

          std::cout << "Current state: " << currBehavior.state << ", possible next states: ";
          for (auto state : possibleStates) {
            std::cout << state << " ";
          }
          std::cout << std::endl;

          // Cache of generated path cache, and its cost.
          std::vector<std::pair<Path, double>> pathCache;

          double targetSpeed = ComputeTargetSpeed(
            currBehavior,
            currCarState,
            prevPath,
            naviMap,
            sensorFusions,
            endPathFrenetPose,
            currBehavior.lane.id);

          for (const BehaviorState &candidateBehaviorState : possibleStates) {
            // NOTE: Goal generation is embeded inside path generator
            // since the goal of this project is just to keep the car running
            std::cout << "Generating candidate path for behavior=" << candidateBehaviorState << std::endl;

            Path newPath = GeneratePath(
              candidateBehaviorState,
              targetSpeed,
              currBehavior,
              currCarState,
              prevPath,
              naviMap,
              endPathFrenetPose,
              2
            );

            // double score = EvaluatePath(
            //   newPath,
            //   currvBehavior,
            //   sensorFusions
            // );
            pathCache.push_back({newPath, 1.0});
          }

          decltype(pathCache)::iterator bestPathItr = std::min_element(
            pathCache.begin(), pathCache.end(),
            [] (const std::pair<Path, double> &e1, const std::pair<Path, double> &e2) {
              return e1.second > e2.second;
            }
          );
          size_t bestIndex = std::distance(pathCache.begin(), bestPathItr);
          Path newPath = pathCache[bestIndex].first;

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
