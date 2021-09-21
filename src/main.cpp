#include <uWS/uWS.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <boost/log/trivial.hpp>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "planner.h"
#include "json.hpp"
#include "spline.h"

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

  // Lane reference for navigation, initial value is the initial lane,
  // will be updated over time.
  // In this project, the land id can be 0, 1, 2 (3 lanes).
  int laneIdReference = 1;

  // Speed reference for navidation, initial value is the initial speed,
  // will be updated over time, unit is in mile (simulator is using this unit, Orz).
  double speedReference = 0.0;

  // Main event loop callback when we receive something from the simulator.
  // These parameters are specific to uWS communication.
  h.onMessage(
    [&naviMap, &laneIdReference, &speedReference] (
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
            {j[1]["x"], j[1]["y"], deg2rad(j[1]["yaw"])},
            {j[1]["s"], j[1]["d"]},
            j[1]["speed"]
          );
          BOOST_LOG_TRIVIAL(info) << currCarState;
 
          // Previous path data given to the Planner
          Path prevPath = ConvertXYToPath(
            j[1]["previous_path_x"],
            j[1]["previous_path_y"]
          );

          const size_t prevPathSize = prevPath.size();

          // Previous path's end s and d values
          double endPathSValue = j[1]["end_path_s"];
          double endPathDValue = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          const ProbeData sensorFusion = j[1]["sensor_fusion"];

          //
          // Behavior generation
          //
          BehaviorState nextBehaviorState(laneIdReference, speedReference);

          //
          // Path generation based on next behavior
          //
          json msgJson;
          Path newPath = GeneratePath(
            nextBehaviorState.laneId,
            nextBehaviorState.speed,
            currCarState,
            prevPath,
            naviMap,
            2
          );

          // Path newPath = GeneratePath(currCarState, naviMap);

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
