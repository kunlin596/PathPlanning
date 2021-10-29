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

namespace pathplanning {

/**
 * @brief      Perception data format from simulator
 */
struct Perception
{
  int id = -1;
  std::array<double, 2> xy;
  std::array<double, 2> vel; ///< meters/sec
  std::array<double, 2> sd;
  double speed;

  Perception() {}
  Perception(int id,
             double x,
             double y,
             double vx,
             double vy,
             double s,
             double d)
    : id(id)
    , xy({ x, y })
    , vel({ vx, vy })
    , sd({ s, d })
  {
    speed = std::sqrt(vel[0] * vel[0] + vel[1] * vel[1]);
  }

  virtual ~Perception() {}

  static std::unordered_map<int, Perception> CreatePerceptions(
    const std::vector<std::vector<double>>& data)
  {
    std::unordered_map<int, Perception> perceptions;
    for (size_t i = 0; i < data.size(); ++i) {
      const std::vector<double> d = data[i];
      perceptions[d[0]] = Perception(d[0], d[1], d[2], d[3], d[4], d[5], d[6]);
    }
    return perceptions;
  }

  friend std::ostream& operator<<(std::ostream& out,
                                  const pathplanning::Perception& perception);
};

using Perceptions = std::unordered_map<int, Perception>;

} // namespace pathplanning

// IO functions

inline std::ostream&
operator<<(std::ostream& out, const pathplanning::Perception& perception)
{
  return out << fmt::format(
           "Perception=(id={:3d}, xy=[{:7.3f}, {:7.3f}], vel=[{:7.3f}, "
           "{:7.3f}], sd=[{:7.3f}, {:7.3f}])",
           perception.id,
           perception.xy[0],
           perception.xy[1],
           perception.vel[0],
           perception.vel[1],
           perception.sd[0],
           perception.sd[1]);
}

inline std::ostream&
operator<<(std::ostream& out, const pathplanning::Perceptions& perceptions)
{
  out << std::string("{\n");
  for (const auto& p : perceptions) {
    out << fmt::format("  {:3d}: {:s},\n", p.first, p.second);
  }
  return out << std::string("}\n");
}

#endif
