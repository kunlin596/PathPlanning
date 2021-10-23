#ifndef PATHPLANNING_SYSTEM_H
#define PATHPLANNING_SYSTEM_H

#include "behaviorplanner.h"
#include "ptg.h"

namespace pathplanning {

/**
 * @brief      This class describes a system.
 */
class System {
 public:
  System() { _pMap = Map::CreateMap(); };
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

  void UpdatePerception(const std::string &) {}

 private:
  Map::Ptr _pMap;
  std::unique_ptr<BehaviorPlanner> _pBehaviorPlanner;
  std::unique_ptr<PolynomialTrajectoryGenerator> _pPathGenerator;
};

}  // namespace pathplanning

#endif
