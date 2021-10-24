#include "vehicle.h"

#include "configuration.h"

namespace pathplanning {

void Vehicle::UpdateFromPerception(const Map::ConstPtr &pMap,
                                   const Perception &perception) {
  // TODO
}

Vehicle Vehicle::CreateFromPerception(const Map::ConstPtr &pMap,
                                      const Perception &perception) {
  int id = perception.id;
  VehicleConfiguration conf;
  conf.sPos = perception.sd[0];
  conf.dPos = perception.sd[1];

  std::array<double, 2> sd2 =
      pMap->GetSD(perception.xy[0] + perception.vel[0],
                  perception.xy[1] + perception.vel[1],
                  std::atan2(perception.vel[1], perception.vel[0]));

  conf.sVel = (sd2[0] - conf.sPos);
  conf.dVel = (sd2[1] - conf.dPos);
  // Assume constant accelaration

  return Vehicle(id, conf);
}

Vehicle Vehicle::CreateFromEgo(const Map::ConstPtr &pMap, const Ego &ego) {
  double theta = deg2rad(ego.yaw);
  double x2 = ego.x - std::cos(theta) * ego.speed * Configuration::TIME_STEP;
  double y2 = ego.y - std::sin(theta) * ego.speed * Configuration::TIME_STEP;
  std::array<double, 2> sd2 = pMap->GetSD(ego.x, ego.y, theta);
  // Assume constant accelaration
  double sDot = (sd2[0] - ego.s) / Configuration::TIME_STEP;
  double dDot = (sd2[1] - ego.d) / Configuration::TIME_STEP;
  return Vehicle(std::numeric_limits<int>::max(),
                 VehicleConfiguration(ego.s, sDot, 0.0, ego.d, dDot, 0.0));
}

}  // namespace pathplanning
