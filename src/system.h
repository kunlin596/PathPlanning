#ifndef PATHPLANNING_SYSTEM_H
#define PATHPLANNING_SYSTEM_H

#include <uWS/uWS.h>

#include "behaviorplanner.h"
#include "ptg.h"
#include "tracker.h"

namespace pathplanning {

/**
 * @brief      This class describes a system.
 */
class System {
 public:
  System() {
    _pMap = Map::CreateMap();
    _pHub = std::make_unique<uWS::Hub>();
  };
  virtual ~System(){};

  /**
   * @brief      System options/parameters
   */
  struct Options {};

  /**
   * @brief      Initialize the system with input configuration file
   *
   * @param[in]  configFilename  The configuration filename
   */
  void Initialize(const std::string &configFilename);

  /**
   * @brief      Reset map
   *
   * @param[in]  filename  The map csv filename
   */
  inline void ResetMap(const std::string &filename) { _pMap->Read(filename); }

  /**
   * @brief      Spin the system once
   *
   * @param[in]  commandString  The command string
   *
   * @return     Result message string
   */
  std::string SpinOnce(const std::string &commandString);

  int Spin();

 private:
  Map::Ptr _pMap;
  std::unique_ptr<BehaviorPlanner> _pBehaviorPlanner;
  std::unique_ptr<Tracker> _pTracker;
  std::unordered_map<int, Vehicle> _pPerceptions;
  std::unique_ptr<PolynomialTrajectoryGenerator> _pPathGenerator;
  std::unique_ptr<uWS::Hub> _pHub;
  int _port = 4567;
};

}  // namespace pathplanning

#endif
