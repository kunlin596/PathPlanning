#ifndef PATHPLANNING_PERCEPTION_H
#define PATHPLANNING_PERCEPTION_H

#include <array>
#include <cmath>
#include <ostream>
#include <unordered_map>
#include <vector>

#include "log.h"
#include "map.h"
#include "vehicle.h"

namespace pathplanning {

/**
 * @brief      Perception data format from simulator
 */
struct Perception
{
  Perception(){};

  Perception(int id, double x, double y, double vx, double vy, double s, double d, const Map& map, const double time)
    : id(id)
    , xy({ x, y })
    , xyVel({ vx, vy })
    , sd({ s, d })
  {
    speed = std::sqrt(xyVel[0] * xyVel[0] + xyVel[1] * xyVel[1]);
    sdVel = ComputeFrenetVelocity(map, xy, xyVel, sd, time);
  }

  virtual ~Perception() {}

  static std::unordered_map<int, Perception> CreatePerceptions(const std::vector<std::vector<double>>& data,
                                                               const Map& map,
                                                               const double time);

  Vehicle GetVehicle() const;

  friend std::ostream& operator<<(std::ostream& out, const Perception& perception);

  int id = -1;
  std::array<double, 2> xy;
  std::array<double, 2> xyVel; ///< meters/sec
  std::array<double, 2> sd;
  std::array<double, 2> sdVel; ///< meters/sec
  double speed;
};

using Perceptions = std::unordered_map<int, Perception>;

std::ostream&
operator<<(std::ostream& out, const Perception& perception);

std::ostream&
operator<<(std::ostream& out, const Perceptions& perceptions);

} // namespace pathplanning

// IO functions
#endif
