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

std::string _EmptyControlMessage() {
  nlohmann::json waypointsJson;
  waypointsJson["next_x"] = std::vector<double>();
  waypointsJson["next_y"] = std::vector<double>();
  return "42[\"control\"," + waypointsJson.dump() + "]";
}

}  // namespace

namespace pathplanning {

void System::Initialize(const std::string &configFilename) {
  _pMap = Map::CreateMap();
  _pHub = std::make_unique<uWS::Hub>();
  _pTracker = std::make_unique<Tracker>(_pMap);
  _pBehaviorPlanner = std::make_unique<BehaviorPlanner>(_pMap);
  _pPathGenerator = std::make_unique<PolynomialTrajectoryGenerator>(_pMap);
  _pEgo = std::make_unique<Ego>();
}

std::string System::SpinOnce(const std::string &commandString) {
  using nlohmann::json;
  using std::cout;
  using std::endl;

  std::string msg;

  // static int cnt = 0;
  // if (cnt < 10) {
  //   ++cnt;
  //   return _EmptyControlMessage();
  // } else {
  //   cnt = 0;
  // }

  static Vehicle cachedProposal;

  if (!commandString.empty()) {
    json commandJson = json::parse(commandString);
    std::string event = commandJson[0].get<std::string>();
    const json &data = commandJson[1];

    if (event == "telemetry") {
      // Similator might not be able to consume all of the points, so it returns
      // the remaining points back to you.
      Waypoints prevPath = Path::ConvertXYToWaypoints(data["previous_path_x"],
                                                      data["previous_path_y"]);

      // NOTE: Ego's localization data (No Noise)
      // ["x"] The car's x position in map coordinates
      // ["y"] The car's y position in map coordinates
      // ["s"] The car's s position in frenet coordinates
      // ["d"] The car's d position in frenet coordinates
      // ["yaw"] The car's yaw angle in the map int degrees
      // ["speed"] The car's speed in MPH

      std::cout << std::endl;
      SPDLOG_INFO("--- telemetry ---");
      Waypoint endPathSD = {data["end_path_s"], data["end_path_d"]};

      _pEgo->Update(data["x"], data["y"], data["s"], data["d"],
                    deg2rad(data["yaw"]), mph2ms(data["speed"]));

      double cacheds = cachedProposal.GetConfiguration().sPos;

      if (cacheds > 0.0) {
        // FIXME: debug purpose
        // assert(std::abs(_pEgo->s - cacheds) < 100.0);
      }

      Vehicle egoVehicle = Vehicle::CreateFromEgo(_pMap, *_pEgo);

      //
      // Process perceptions
      //

      // Create the latest perceptions from input command
      Perceptions perceptions =
          Perception::CreatePerceptions(data["sensor_fusion"]);
      SPDLOG_DEBUG("perceptions={}", perceptions);

      _pTracker->Update(egoVehicle, perceptions);
      Predictions predictions = _pTracker->GeneratePredictions();
      SPDLOG_DEBUG("Generated predictions for {} vihicles", predictions.size());

      //
      // Behavior planning
      //

      const auto successorStates =
          _pBehaviorPlanner->GetSuccessorStates(BehaviorState::kLaneKeeping);

      Vehicle startState = _pPathGenerator->ComputeStartState(
          egoVehicle, _state.cachedTrajectory, prevPath, endPathSD);
      SPDLOG_DEBUG("Computed startState={}", startState.GetConfiguration());

      Vehicle proposal = _pBehaviorPlanner->GenerateProposal(
          startState, prevPath, endPathSD, successorStates, predictions);

      cachedProposal = proposal;

      //
      // Path generation
      //

      SPDLOG_INFO("Generating path for,\nstartState={}\npropoState={}",
                   startState.GetConfiguration(), proposal.GetConfiguration());
      double targetExecutionTime = 1.0;
      Waypoints path;
      JMTTrajectory trajectory;
      std::tie(path, trajectory) = _pPathGenerator->GeneratePath(
          startState, proposal, predictions, targetExecutionTime);

      UpdateCachedTrajectory(trajectory);

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
