#ifndef PATHPLANNING_BEHAVIORPLANNER_H
#define PATHPLANNING_BEHAVIORPLANNER_H

#include "map.h"
#include "ptg.h" // For generating goals for PTG
#include "tracker.h"
#include "traj_evaluator.h"

namespace pathplanning {

/**
 * @brief      States of FSM for behavior planner
 *
 * kLaneKeeping:
 *  - d, stay near center line for lane
 *  - s, drive at target speed when feasible
 *
 * kLeftLaneChange/kRightLaneChange:
 *  - d, move left or right
 *  - s, same rules as keep lane (for initial lane)
 *
 * kLeftLaneChangePreparation/kRightLaneChangePreparation:
 *  - d, stay near cener line for current lane
 *  - s, attempt to match position and speed of "gap" in lane
 *  - signal, activate turning signal
 *
 */
enum class BehaviorState
{
  kReady,
  kConstSpeed,
  kLaneKeeping,
  kLeftLaneChangePreparation,
  kLeftLaneChange,
  kRightLaneChangePreparation,
  kRightLaneChange
};

/**
 * @brief      This class describes a behavior planner.
 *
 * The responsibility of behavior planner is to take in the map, route and
 * perceptions(predictions) and then produce a suggested maneuver.
 *
 * The suggested vechile configuration / maneuvers should be,
 * - feasible
 * - safe
 * - legal
 * - efficient
 *
 * However, the behavior planner is not responsible for
 * - execution details
 * - collision avoidance
 *
 * How does behavior planner works:
 * Behavior planner will take in the map and perceptions and figure out the
 * vechiles that are close to ego. Then close perceptions will be used for
 * generating perdictions.
 *
 * Behavior planner uses FSM in this relatively simple high way scenario, which
 * only contains 8 states (listed above).
 *
 * For current state, there will be a set of possible successor states. For each
 * possible successor state, we are going to generate a rough goal configuration
 * and trajectory, then evaluate each one of them.
 *
 * Finally, the best proposed end configuration to real trajectory generation
 * module.
 */
class BehaviorPlanner
{
public:
  BehaviorPlanner(const Map& map, const Configuration& conf);
  virtual ~BehaviorPlanner() {}

  /**
   * @brief      Generate maneuver proposal trajectory
   */
  JMTTrajectory2d GenerateProposal(const Matrix32d& startKinematics, const TrackedVehicleMap& trackedVehicleMap);

private:
  const Map& _map;
  const Configuration& _conf;
  std::unique_ptr<JMTTrajectoryEvaluator> _pEvaluator;
  BehaviorState _currState = pathplanning::BehaviorState::kReady;
};

std::ostream&
operator<<(std::ostream& out, const BehaviorState& type);

} // namespace pathplanning

#endif
