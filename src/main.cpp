#include <uWS/uWS.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "json.hpp"
#include "map.h"

namespace {

/**
 * @brief      Normalize input JSON string data
 *
 * @param[in]  s     Input data
 *
 * @return     Normalized JSON string
 */
std::string _NormalizeJsonString(std::string s) {
  using std::string;
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}
}

int main(int argc, char ** argv) {
  using nlohmann::json;
  using std::string;
  using std::vector;
  using namespace pathplanning;

  uWS::Hub h;

  Map map;
  map.Read("../data/highway_map.csv");

  // Main event loop callback when we receive something from the simulator.
  // These parameters are specific to uWS communication.
  h.onMessage([&map](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = _NormalizeJsonString(data);

      if (s != "") {
        auto j = json::parse(s);

        std::string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          // TODO
          json msgJson;

          std::vector<double> nextXValues, nextYValues;
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
    std::cout << "Connected!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
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
