#include "perception.h"

namespace pathplanning {
std::ostream&
operator<<(std::ostream& out, const pathplanning::Perception& perception)
{
  return out << fmt::format(
           "Perception=(id={:3d}, xy=[{:7.3f}, {:7.3f}], vel=[{:7.3f}, "
           "{:7.3f}], sd=[{:7.3f}, {:7.3f}])",
           perception.id,
           perception.xy[0],
           perception.xy[1],
           perception.xyVel[0],
           perception.xyVel[1],
           perception.sd[0],
           perception.sd[1]);
}

std::ostream&
operator<<(std::ostream& out, const pathplanning::Perceptions& perceptions)
{
  out << std::string("[\n");
  // for (const auto& p : perceptions) {
  //   out << fmt::format("  {:s},\n", p.second);
  // }
  return out << std::string("]\n");
}

}; // namespace pathplanning
