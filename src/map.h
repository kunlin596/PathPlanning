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
class Map
{
public:
  static constexpr double MAX_FRENET_S = 6945.554;
  static constexpr double LANE_WIDTH = 4.0;
  static constexpr double HALF_LANE_WIDTH = LANE_WIDTH / 2.0;
  static constexpr int NUM_LANES = 3;

  Map();
  Map(const std::string& filename);

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

  // Transform from Frenet s,d coordinates to Cartesian x
  double GetX(double s, double d) const;

  // Transform from Frenet s,d coordinates to Cartesian y
  double GetY(double s, double d) const;

  // Transform from Frenet s,d coordinates to Cartesian x, y
  std::array<double, 2> GetXY(double s, double d) const;

  static inline int GetLaneId(double d)
  {
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
  std::array<double, 5> Get(int index) const
  {
    // TODO: Add boundary checking
    return { _x[index], _y[index], _s[index], _dx[index], _dy[index] };
  }

  static inline double GetLaneCenterD(int laneId) { return HALF_LANE_WIDTH + LANE_WIDTH * static_cast<double>(laneId); }

  static inline std::array<double, 2> GetRoadBoundary() { return { 0.0, NUM_LANES * LANE_WIDTH }; }

  static inline std::array<double, 2> GetLaneBoundary(int laneId)
  {
    double centerD = GetLaneCenterD(laneId);
    return { centerD - HALF_LANE_WIDTH, centerD + HALF_LANE_WIDTH };
  }

  static inline bool IsInRoad(double d)
  {
    static const auto roadBoundary = GetRoadBoundary();
    return roadBoundary[0] < d and d < roadBoundary[1];
  }

  static inline bool IsInLane(double d, int laneId)
  {
    return (static_cast<double>(laneId) * LANE_WIDTH) < d and d < (static_cast<double>(laneId + 1) * LANE_WIDTH);
  }

  void Read(const std::string& filename);

  static std::shared_ptr<Map> CreateMap() { return std::make_shared<Map>(); }
  static std::shared_ptr<Map> CreateMap(const std::string filename) { return std::make_shared<Map>(filename); }

  using Ptr = std::shared_ptr<Map>;
  using ConstPtr = std::shared_ptr<const Map>;

private:
  std::vector<double> _x;  ///< All x coords
  std::vector<double> _y;  ///< All y coords
  std::vector<double> _s;  ///< All s coords
  std::vector<double> _dx; ///< All x components of tangent unit direction vector
  std::vector<double> _dy; ///< All y components of tangent unit direction vector

  /**
   * @brief      For hiding spline implemenetation
   *
   * Spline is implemented in a anonymous namespace, which will cause the
   * warning below when put in header file.
   *
   * warning: ‘pathplanning::Map’ has a
   * field ‘pathplanning::Map::_sySpline’ whose type uses the anonymous
   * namespace [-Wsubobject-linkage]
   *
   * Here I didn't bother to put all members into pimpl pattern, but once used,
   * I should, but just being lazy here.
   */
  struct Impl;
  std::shared_ptr<Impl> _pImpl;
};

std::array<double, 2>
ComputeFrenetVelocity(const Map& map,
                      const std::array<double, 2>& pos,
                      const std::array<double, 2>& vel,
                      const std::array<double, 2>& sd,
                      const double dt);

} // namespace pathplanning

#endif
