#ifndef PATHPLANNING_SYSTEM_H
#define PATHPLANNING_SYSTEM_H

#include <uWS/uWS.h>

#include "ptg.h"
#include "tracker.h"

namespace pathplanning {

/**
 * @brief      This class describes a system.
 */
class System
{
public:
  System() { Initialize(); };
  virtual ~System(){};

  struct State
  {
    JMTTrajectory2d cachedTrajectory;
  };

  /**
   * @brief      System options/parameters
   */
  struct Options
  {};

  /**
   * @brief      Initialize the system with input configuration file
   *
   * @param[in]  configFilename  The configuration filename
   */
  void Initialize(const std::string& configFilename = "");

  /**
   * @brief      Reset map
   *
   * @param[in]  filename  The map csv filename
   */
  inline void ResetMap(const std::string& filename) { _pMap->Read(filename); }

  /**
   * @brief      Spin the system once
   *
   * @param[in]  commandString  The command string
   *
   * @return     Result message string
   */
  std::string SpinOnce(const std::string& commandString);

  /**
   * @brief      Spin the planning server
   *
   * @return     Status code
   */
  int Spin();

  void UpdateCachedTrajectory(const JMTTrajectory2d& traj) { _state.cachedTrajectory = traj; }

  State GetState() const { return _state; }

private:
  Map::Ptr _pMap;
  std::unique_ptr<Tracker> _pTracker;
  std::unique_ptr<PolynomialTrajectoryGenerator> _pPathGenerator;
  std::unique_ptr<uWS::Hub> _pHub;
  std::unique_ptr<Ego> _pEgo;
  std::unique_ptr<Configuration> _pConf;
  int _port = 4567;
  State _state;
};

} // namespace pathplanning

#endif
