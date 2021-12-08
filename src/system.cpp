#include "system.h"

#include "json.hpp"
#include "log.h"
#include "utils.h"

namespace {
using namespace pathplanning;
/**
 * @brief      Normalize input JSON string data
 *
 * @param[in]  s     Input data
 *
 * @return     Normalized JSON string
 */
std::string
_NormalizeJsonString(std::string s)
{
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

std::string
_EmptyControlMessage(const Waypoints& prevPath)
{
  nlohmann::json waypointsJson;
  std::tie(waypointsJson["next_x"], waypointsJson["next_y"]) = Path::ConvertWaypointsToXY(prevPath);
  return "42[\"control\"," + waypointsJson.dump() + "]";
}

} // namespace

namespace pathplanning {

void
System::Initialize(const std::string& configFileName)
{
  _pConf = std::make_unique<Configuration>(configFileName);
  _port = _pConf->serverPort;
  _pMap = Map::CreateMap();
  _pHub = std::make_unique<uWS::Hub>();
  _pTracker = std::make_unique<Tracker>(*_pMap, *_pConf);
  _pPathGenerator = std::make_unique<PolynomialTrajectoryGenerator>(*_pMap, *_pConf);
  _pEgo = std::make_unique<Ego>();
}

std::string
System::SpinOnce(const std::string& commandString)
{
  using nlohmann::json;
  using std::cout;
  using std::endl;

  std::string msg;

  // static int step = 0;

  if (!commandString.empty()) {
    json commandJson = json::parse(commandString);
    std::string event = commandJson[0].get<std::string>();
    const json& data = commandJson[1];

    if (event == "telemetry") {

      // Simulator might not be able to consume all of the points, so it returns
      // the remaining points back to you.
      Waypoints prevPath = Path::ConvertXYToWaypoints(data["previous_path_x"], data["previous_path_y"]);

      // NOTE: Ego's localization data (No Noise)
      // ["x"] The car's x position in map coordinates
      // ["y"] The car's y position in map coordinates
      // ["s"] The car's s position in Frenet coordinates
      // ["d"] The car's d position in Frenet coordinates
      // ["yaw"] The car's yaw angle in the map int degrees
      // ["speed"] The car's speed in MPH

      Waypoint endPathSD = { data["end_path_s"], data["end_path_d"] };

      Ego& ego = *_pEgo;
      Map& map = *_pMap;
      Configuration& conf = *_pConf;

      ego.Update(data["x"],
                 data["y"],
                 std::fmod(data["s"], Map::MAX_FRENET_S),
                 data["d"],
                 Deg2Rad(data["yaw"]),
                 Mph2Mps(data["speed"]),
                 map,
                 conf);

      // Create the latest perceptions from input command
      // Velocity is already meters per seconds no need to convert
      _pTracker->Update(ego, Perception::CreatePerceptions(data["sensor_fusion"], map, conf.simulatorTimeStep));

      double executedTime = 0.0;
      Matrix32d startState = ego.GetKinematics(0.0).topRows<3>();
      if (prevPath.size() > 0) {
        executedTime = (_state.prevPathSize - prevPath.size()) * conf.simulatorTimeStep;
        startState = _state.cachedTrajectory(executedTime).topRows<3>();
      }

      //
      // Path generation
      //
      json log;

      // Find trajectory
      Ego futureEgo = ego;
      futureEgo.SetKinematics(startState);
      JMTTrajectory2d trajectory = _pPathGenerator->GenerateTrajectory(futureEgo, _pTracker->GetVehicles());
      if (!trajectory.GetIsValid()) {
        SPDLOG_ERROR("Planning failed, returning prev path.");
        return _EmptyControlMessage(prevPath);
      }

      Waypoints path;

      Matrix62d p;
      double currTime = 0.0;
      while (currTime < trajectory.GetTime()) {
        p = trajectory(currTime);
        path.push_back(_pMap->GetXY(p(0, 0), p(0, 1)));
        currTime += _pConf->simulatorTimeStep;
      }

      // Construct result message
      json waypointsJson;
      std::tie(waypointsJson["next_x"], waypointsJson["next_y"]) = Path::ConvertWaypointsToXY(path);

      UpdateCachedTrajectory(trajectory, path.size());

      msg = "42[\"control\"," + waypointsJson.dump() + "]";
    }
  } else {
    msg = "42[\"manual\",{}]";
  }

  return msg;
}

int
System::Spin()
{
  // Main event loop callback when we receive something from the simulator.
  // These parameters are specific to uWS communication.
  _pHub->onMessage([this](uWS::WebSocket<uWS::SERVER> ws, char* data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      std::string msg = SpinOnce(_NormalizeJsonString(data));
      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
    } // end websocket if
  }); // end _pHub->onMessage

  _pHub->onConnection([this](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) { SPDLOG_INFO("Connected."); });

  _pHub->onDisconnection([this](uWS::WebSocket<uWS::SERVER> ws, int code, char* message, size_t length) {
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

} // namespace pathplanning
