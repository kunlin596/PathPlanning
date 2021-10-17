#ifndef PATHPLANNING_PATH_H
#define PATHPLANNING_PATH_H

#include "map.h"

namespace pathplanning {

/**
 * @brief      This class describes a system.
 */
class System {
 public:
  System(){};
  virtual ~System(){};

  /**
   * @brief      System options/parameters
   */
  struct Options
  {

  };

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
  inline void ResetMap(const std::string &filename) { _map.Read(filename); }

 private:
  Map _map;
};

}  // namespace pathplanning

#endif
