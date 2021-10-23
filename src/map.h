#ifndef PATHPLANNING_MAP_H
#define PATHPLANNING_MAP_H

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "path.h"

namespace pathplanning {

/**
 * @brief      This class describes a map for navigation.
 */
class Map {
 public:
  Map() {}
  Map(const std::string &filename) { Read(filename); }

  virtual ~Map() {}

  // Calculate closest waypoint to current x, y position

  int GetClosestWaypoint(double x, double y) const;

  // Returns next waypoint of the closest waypoint
  int GetNextWaypoint(double x, double y, double theta) const;

  /**
   * @brief      Gets the Frenet sd coordinates.
   *
   * @param[in]  x      x coordinate
   * @param[in]  y      y coordinate
   * @param[in]  theta  The theta in radian
   *
   * @return     The Frenet sd coordinates.
   */
  std::array<double, 2> GetSD(double x, double y, double theta) const;

  // Transform from Frenet s,d coordinates to Cartesian x,y
  std::array<double, 2> GetXY(double s, double d) const;

  inline double GetDValueFromLandId(int laneId) {
    static constexpr double laneWidth = 4.0;      // meter
    static constexpr double halfLaneWidth = 2.0;  // meter
    return halfLaneWidth + laneWidth * static_cast<double>(laneId);
  }

  void Read(const std::string &filename);

  static constexpr double MaxS = 6945.554;

  static std::shared_ptr<Map> CreateMap() { return std::make_shared<Map>(); }
  static std::shared_ptr<Map> CreateMap(const std::string filename) {
    return std::make_shared<Map>(filename);
  }

  using Ptr = std::shared_ptr<Map>;
  using ConstPtr = std::shared_ptr<const Map>;

 private:
  std::vector<double> _x;
  std::vector<double> _y;
  std::vector<double> _s;
  std::vector<double> _dx;
  std::vector<double> _dy;
};

}  // namespace pathplanning

#endif
