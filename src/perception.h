#ifndef PATHPLANNING_PERCEPTION_H
#define PATHPLANNING_PERCEPTION_H

#include <array>
#include <boost/format.hpp>
#include <cmath>
#include <map>
#include <ostream>
#include <unordered_map>
#include <vector>

namespace pathplanning {

/**
 * @brief      Perception data format from simulator
 */
struct Perception {
  int id = -1;
  std::array<double, 2> xy;
  std::array<double, 2> vel;
  std::array<double, 2> sd;
  double speed;

  Perception() {}
  Perception(int id, double x, double y, double vx, double vy, double s,
             double d)
      : id(id), xy({x, y}), vel({vx, vy}), sd({s, d}) {
    speed = std::sqrt(vel[0] * vel[0] + vel[1] * vel[1]);
  }

  virtual ~Perception() {}

  static std::unordered_map<int, Perception> CreatePerceptions(
      const std::vector<std::vector<double>> &data) {
    std::unordered_map<int, Perception> perceptions;
    for (size_t i = 0; i < data.size(); ++i) {
      const std::vector<double> d = data[i];
      perceptions[d[0]] = Perception(d[0], d[1], d[2], d[3], d[4], d[5], d[6]);
    }
    return perceptions;
  }
};

using Perceptions = std::unordered_map<int, Perception>;

// IO functions

}  // namespace pathplanning

inline std::ostream &operator<<(std::ostream &out,
                                const pathplanning::Perception &perception) {
  out << (boost::format(
              "Perception=(id=%d, xy=[%.3f, %.3f], vel=[%.3f, %.3f], sd=[%.3f, "
              "%.3f])") %
          perception.id % perception.xy[0] % perception.xy[1] %
          perception.vel[0] % perception.vel[1] % perception.sd[0] %
          perception.sd[1])
             .str();
  return out;
}

inline std::ostream &operator<<(std::ostream &out,
                                const pathplanning::Perceptions &perceptions) {
  using std::endl;
  out << std::string("{") << endl;
  for (const auto &perception : perceptions) {
    out << std::string("  ") << perception.first << ": " << perception.second
        << std::endl;
  }
  out << std::string("}") << endl;
  return out;
}

#endif
