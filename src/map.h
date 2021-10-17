#ifndef PATHPLANNING_MAP_H
#define PATHPLANNING_MAP_H

#include "path.h"

#include <string>
#include <array>
#include <vector>

namespace pathplanning {

/**
 * @brief      This class describes a map for navigation.
 */
class Map {
public:
  // Calculate closest waypoint to current x, y position
  int GetClosestWaypoint(double x, double y);

  // Returns next waypoint of the closest waypoint
  int GetNextWaypoint(double x, double y, double theta);

  // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
  std::array<double, 2> GetSD(double x, double y, double theta);

  // Transform from Frenet s,d coordinates to Cartesian x,y
  std::array<double, 2> GetXY(double s, double d);

  inline double GetDValueFromLandId(int laneId) {
    static constexpr double laneWidth = 4.0;      // meter
    static constexpr double halfLaneWidth = 2.0;  // meter
    return halfLaneWidth + laneWidth * static_cast<double>(laneId);
  }

  void Read(const std::string &filename);

  static constexpr double MaxS = 6945.554;

private:
  std::vector<double> _x;
  std::vector<double> _y;
  std::vector<double> _s;
  std::vector<double> _dx;
  std::vector<double> _dy;
};

}  // namespace pathplanning

#endif
