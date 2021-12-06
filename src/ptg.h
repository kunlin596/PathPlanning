#ifndef PATHPLANNING_PTG_H
#define PATHPLANNING_PTG_H

#include <boost/optional.hpp>

#include "jmt.h"
#include "json.hpp"
#include "tracker.h"

namespace pathplanning {

using namespace nlohmann;

enum class LaneType
{
  kEgo,
  kLeft,
  kRight,
};

enum class LongitudinalManeuverType
{
  kFollowing,
  kCruising,
  kStopping,
};

enum class LateralManeuverType
{
  kLaneKeeping,
  kLeftLaneChanging,
  kRightLaneChanging,
};

/**
 * @brief      This class describes a polynomial trajectory generator.
 */
class PolynomialTrajectoryGenerator
{
public:
  explicit PolynomialTrajectoryGenerator(const Map& map, const Configuration& conf);
  virtual ~PolynomialTrajectoryGenerator() {}

  JMTTrajectory2d GenerataTrajectory(const Ego& ego, const TrackedVehicleMap& trackedVehicleMap);

private:
  /**
   * @brief      Generate 1d lateral trajectory for 1 lane
   *
   * @param[in]  latBehavior   The lat behavior
   * @param[in]  ego           The ego
   * @param      trajectories  The trajectories
   */
  void _GenerateLatTrajectory(const LateralManeuverType& latBehavior,
                              const Ego& ego,
                              std::vector<JMTTrajectory1d>& trajectories);

  /**
   * @brief      Generate 1d longitudinal trajectory for 1 lane
   *
   * @param[in]  lonBehavior   The lon behavior
   * @param[in]  ego           The ego
   * @param[in]  vehicle       The vehicle
   * @param      trajectories  The trajectories
   */
  void _GenerateLonTrajectory(const LongitudinalManeuverType& lonBehavior,
                              const Ego& ego,
                              const Vehicle& vehicle,
                              std::vector<JMTTrajectory1d>& trajectories);

  /**
   * @brief      Generate velocity keeping trajectory
   *
   * @param[in]  ego           The ego
   * @param      trajectories  The trajectories
   */
  void _GenerateCrusingTrajectory(const Ego& ego, std::vector<JMTTrajectory1d>& trajectories);

  /**
   * @brief      Generate Vehicle following trajectory
   *
   * @param[in]  ego           The ego
   * @param[in]  vehicle       The vehicle
   * @param      trajectories  The trajectories
   */
  void _GenerateVehicleFollowingTrajectory(const Ego& ego,
                                           const Vehicle& vehicle,
                                           std::vector<JMTTrajectory1d>& trajectories);

  /**
   * @brief      Generate stopping trajectory
   *
   * @param[in]  ego           The ego
   * @param      trajectories  The trajectories
   * @param      costs         The costs
   */
  void _GenerateStoppingTrajectory(const Ego& ego, std::vector<JMTTrajectory1d>& trajectories);

  Matrix32d _ComputeStartStatePy(const Vehicle& ego, const JMTTrajectory2d& prevTraj, const Waypoints& prevPath);

  JMTTrajectory2d _GenerataTrajectoryPy(const Ego& ego, const TrackedVehicleMap& trackedVehicleMap);

  const Configuration& _conf;
  int _counter = 0;
  const Map& _map;
  std::unique_ptr<JMTTrajectoryEvaluator> _pEvaluator;
};

std::ostream&
operator<<(std::ostream& out, const LongitudinalManeuverType& type);

std::ostream&
operator<<(std::ostream& out, const LateralManeuverType& type);

}; // namespace pathplanning

#endif
