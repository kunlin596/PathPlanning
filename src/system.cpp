#include "system.h"

#include "json.hpp"

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
}  // namespace

namespace pathplanning {

void System::Initialize(const std::string &configFilename) {
  _pMap = Map::CreateMap();
  _pHub = std::make_unique<uWS::Hub>();
  _pTracker = std::make_unique<Tracker>(_pMap);
}

std::string System::SpinOnce(const std::string &commandString) {
  using nlohmann::json;

  std::string msg;

  if (!commandString.empty()) {
    json commandJson = json::parse(commandString);
    std::string event = commandJson[0].get<std::string>();

    if (event == "telemetry") {
      json waypointsJson;

      // Create the latest perceptions from input command
      Perceptions perceptions =
          Perception::CreatePerceptions(commandJson[1]["sensor_fusion"]);

      _pTracker->Update(perceptions);
      Predictions predictions = _pTracker->GeneratePredictions();
    
      for (const auto &pred : predictions) {
        for (const auto &v : pred.second) {
          std::cout << v << std::endl;
        }
      }

      std::vector<double> nextXValues, nextYValues;
      waypointsJson["next_x"] = nextXValues;
      waypointsJson["next_y"] = nextYValues;

      msg = "42[\"control\"," + waypointsJson.dump() + "]";
    }
  } else {
    msg = "42[\"manual\",{}]";
  }

  return msg;
}

int System::Spin() {
  // Main event loop callback when we receive something from the simulator.
  // These parameters are specific to uWS communication.
  _pHub->onMessage([this](uWS::WebSocket<uWS::SERVER> ws, char *data,
                          size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      std::string msg = SpinOnce(_NormalizeJsonString(data));
      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
    }  // end websocket if
  });  // end _pHub->onMessage

  _pHub->onConnection(
      [this](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!" << std::endl;
      });

  _pHub->onDisconnection([this](uWS::WebSocket<uWS::SERVER> ws, int code,
                                char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  if (_pHub->listen(_port)) {
    std::cout << "Listening to port " << _port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  _pHub->run();
  return 0;
}

}  // namespace pathplanning
