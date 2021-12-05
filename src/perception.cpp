#include "perception.h"

namespace pathplanning {

std::unordered_map<int, Perception>
Perception::CreatePerceptions(const std::vector<std::vector<double>>& data, const Map& map, const double time)
{
  std::unordered_map<int, Perception> perceptions;
  for (size_t i = 0; i < data.size(); ++i) {
    const std::vector<double> d = data[i];
    if (d[6] > 0) {
      perceptions[d[0]] = Perception(d[0], d[1], d[2], d[3], d[4], d[5], d[6], map, time);
    }
  }
  return perceptions;
}

Vehicle
Perception::GetVehicle() const
{
  Matrix32d kinematics = Matrix32d::Zero();
  kinematics(0, 0) = sd[0];
  kinematics(0, 1) = sd[1];
  kinematics(1, 0) = sdVel[0];
  kinematics(1, 1) = sdVel[1];
  // Assume other higher order terms are all zeros.
  return Vehicle(id, kinematics);
}

std::ostream&
operator<<(std::ostream& out, const pathplanning::Perception& perception)
{
  return out << fmt::format("Perception=(id={:3d}, xy=[{:7.3f}, {:7.3f}], vel=[{:7.3f}, "
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
