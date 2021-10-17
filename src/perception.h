#ifndef PATHPLANNING_PERCEPTION_H
#define PATHPLANNING_PERCEPTION_H

#include <vector>
#include <array>
#include <map>
#include <cmath>

namespace pathplanning {

struct Perception {
  int id;
  std::array<double, 2> xy;
  std::array<double, 2> vel;
  std::array<double, 2> sd;
  double speed;

  Perception(int id, double x, double y, double vx, double vy, double s,
             double d)
      : id(id), xy({x, y}), vel({vx, vy}), sd({s, d}) {
    speed = std::sqrt(vel[0] * vel[0] + vel[1] * vel[1]);
  }
};

class PerceptionManager {
 public:

  /**
   * @brief      Update perceptions caches
   *
   * @param[in]  rawdata  The rawdata
   */
  void Update(const std::vector<std::vector<double>> &rawdata);

 private:
  std::map<int, std::vector<Perception>> _perceptions;
};

}  // namespace pathplanning

#endif
