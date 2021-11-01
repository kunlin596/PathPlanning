#ifndef PATHPLANNING_PERCEPTION_H
#define PATHPLANNING_PERCEPTION_H

#include <array>
#include <boost/format.hpp>
#include <cmath>
#include <map>
#include <ostream>
#include <unordered_map>
#include <vector>

#include "log.h"
#include "map.h"

namespace pathplanning {

/**
 * @brief      Perception data format from simulator
 */
struct Perception
{
  int id = -1;
  std::array<double, 2> xy;
  std::array<double, 2> xyVel; ///< meters/sec
  std::array<double, 2> sd;
  std::array<double, 2> sdVel; ///< meters/sec
  double speed;

  Perception() {}
  Perception(int id,
             double x,
             double y,
             double vx,
             double vy,
             double s,
             double d,
             const Map::ConstPtr& pMap,
             const double time)
    : id(id)
    , xy({ x, y })
    , xyVel({ vx, vy })
    , sd({ s, d })
  {
    speed = std::sqrt(xyVel[0] * xyVel[0] + xyVel[1] * xyVel[1]);
    sdVel = ComputeSDVelocity(pMap, xy, xyVel, sd, time);
  }

  virtual ~Perception() {}

  static std::unordered_map<int, Perception> CreatePerceptions(
    const std::vector<std::vector<double>>& data,
    const Map::ConstPtr& pMap,
    const double time)
  {
    std::unordered_map<int, Perception> perceptions;
    for (size_t i = 0; i < data.size(); ++i) {
      const std::vector<double> d = data[i];
      perceptions[d[0]] =
        Perception(d[0], d[1], d[2], d[3], d[4], d[5], d[6], pMap, time);
    }
    return perceptions;
  }

  friend std::ostream& operator<<(std::ostream& out,
                                  const Perception& perception);
};

using Perceptions = std::unordered_map<int, Perception>;

std::ostream&
operator<<(std::ostream& out, const Perception& perception);

std::ostream&
operator<<(std::ostream& out, const Perceptions& perceptions);

} // namespace pathplanning

// IO functions
#endif
