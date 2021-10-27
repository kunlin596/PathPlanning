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
  static constexpr double MaxS = 6945.554;
  static constexpr double LANE_WIDTH = 4.0;
  static constexpr double HALF_LANE_WIDTH = LANE_WIDTH / 2.0;
  static constexpr int NUM_LANES = 3;

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

  static inline int GetLaneId(double d) {
    for (int i = 0; i < NUM_LANES; ++i) {
      if (Map::IsInLane(d, i)) {
        return i;
      }
    }
    return -1;
  }

  /**
   * @brief      Gets the way point in map.
   *
   * @param[in]  index  The index
   *
   * @return     Waypoint array
   */
  std::array<double, 5> Get(int index) const {
    // TODO: Add boundary checking
    return {_x[index], _y[index], _s[index], _dx[index], _dy[index]};
  }

  static inline double GetLaneCenterD(int laneId) {
    return HALF_LANE_WIDTH + LANE_WIDTH * static_cast<double>(laneId);
  }

  static inline bool IsInLane(double d, int laneId) {
    return (static_cast<double>(laneId) * LANE_WIDTH) < d and
           d < (static_cast<double>(laneId + 1) * LANE_WIDTH);
  }

  void Read(const std::string &filename);

  static std::shared_ptr<Map> CreateMap() { return std::make_shared<Map>(); }
  static std::shared_ptr<Map> CreateMap(const std::string filename) {
    return std::make_shared<Map>(filename);
  }

  using Ptr = std::shared_ptr<Map>;
  using ConstPtr = std::shared_ptr<const Map>;

 private:
  std::vector<double> _x;  ///< All x coords
  std::vector<double> _y;  ///< All y coords
  std::vector<double> _s;  ///< All s coords
  std::vector<double>
      _dx;  ///< All x components of tangent unit direction vector
  std::vector<double>
      _dy;  ///< All y components of tangent unit direction vector
};

}  // namespace pathplanning

#endif
