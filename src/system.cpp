#include "system.h"

#include "json.hpp"
#include "log.h"

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
  _pBehaviorPlanner = std::make_unique<BehaviorPlanner>(_pMap);
  _pPathGenerator = std::make_unique<PolynomialTrajectoryGenerator>(_pMap);
}

std::string System::SpinOnce(const std::string &commandString) {
  using nlohmann::json;
  using std::cout;
  using std::endl;

  std::string msg;

  if (!commandString.empty()) {
    json commandJson = json::parse(commandString);
    std::string event = commandJson[0].get<std::string>();
    const json &data = commandJson[1];

    if (event == "telemetry") {
      Waypoints previousPath = Path::ConvertXYToWaypoints(
          data["previous_path_x"], data["previous_path_y"]);
      Waypoint expectedEndPathSD = {data["end_path_s"], data["end_path_d"]};

      Ego ego(data["x"], data["y"], data["s"], data["d"], data["yaw"],
              data["speed"]);

      Vehicle egoVehicle = Vehicle::CreateFromEgo(_pMap, ego);

      //
      // Process perceptions
      //

      // Create the latest perceptions from input command
      Perceptions perceptions =
          Perception::CreatePerceptions(data["sensor_fusion"]);
      SPDLOG_DEBUG("perceptions={}", perceptions);

      _pTracker->Update(egoVehicle, perceptions);
      Predictions predictions = _pTracker->GeneratePredictions();
      // SPDLOG_INFO("Generated predictions for {} vihicles.",
      // predictions.size());

      //
      // Behavior planning
      //
      const auto successorStates =
          _pBehaviorPlanner->GetSuccessorStates(BehaviorState::kLaneKeeping);

      Vehicle proposal = _pBehaviorPlanner->GenerateProposal(
          egoVehicle, successorStates, predictions);
      SPDLOG_INFO("ego={}, proposal={}", egoVehicle, proposal);

      //
      // Path generation
      //
      auto path =
          _pPathGenerator->GeneratePath(egoVehicle, proposal, predictions);
      SPDLOG_INFO("path={}", path);

      //
      // Construct result message
      //
      json waypointsJson;
      std::tie(waypointsJson["next_x"], waypointsJson["next_y"]) =
          Path::ConvertWaypointsToXY(path);

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
        SPDLOG_INFO("Connected.");
      });

  _pHub->onDisconnection([this](uWS::WebSocket<uWS::SERVER> ws, int code,
                                char *message, size_t length) {
    ws.close();
    SPDLOG_INFO("Disconnected.");
  });

  if (_pHub->listen(_port)) {
    SPDLOG_INFO("Listening to port {}.", _port);
  } else {
    SPDLOG_INFO("Failed to listen to port {}.", _port);
    return -1;
  }

  SPDLOG_INFO("Starting planning server.");
  _pHub->run();
  return 0;
}

}  // namespace pathplanning
