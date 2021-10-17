#ifndef PATHPLANNING_BEHAVIORPLANNER_H
#define PATHPLANNING_BEHAVIORPLANNER_H

#include <vector>

#include "map.h"

namespace pathplanning {

/**
 * @brief      Collection of cost functions
 */
struct CostFunctions {
  static double GetGoalDistanceCost(int goalLaneId, int intendedLaneId,
                                    int finalLaneId, double distanceToGoal);

  static double GetInefficiencyCost(int targetSpeed, int intendedLane,
                                    int finalLaneId,
                                    const std::vector<double> &laneSpeeds);
};

/**
 * @brief      Output structure of Behacior planner
 */
struct Goal {
  int targetLaneId;
  int targetLeadingVehicleId;
  double targetSpeed;
  double secondsToReachTarget;
};

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
enum class BehaviorState : uint8_t {
  kLaneKeeping = 0,
  kLeftLaneChangePreparation,
  kLeftLaneChange,
  kRightLaneChangePreparation,
  kRightLaneChange
};

/**
 * @brief      This class describes a behavior planner.
 */
class BehaviorPlanner {
 public:
  BehaviorPlanner(const Map &map) : _map(map) {}
  virtual ~BehaviorPlanner() {}

  /**
   * @brief      Gets the successor states.
   *
   * @param[in]  state  The state
   *
   * @return     The successor states.
   */
  std::vector<BehaviorState> GetSuccessorStates(const BehaviorState &state);

  void ChooseNextState();

 private:
  const Map &_map;
};

}  // namespace pathplanning

#endif
