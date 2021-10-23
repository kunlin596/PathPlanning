#ifndef PATHPLANNING_BEHAVIORPLANNER_H
#define PATHPLANNING_BEHAVIORPLANNER_H

#include "map.h"
#include "ptg.h"  // For generating goals for PTG

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
enum class BehaviorState : uint8_t {
  kConstSpeed = 0,
  kLaneKeeping,
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
  BehaviorPlanner(const Map::ConstPtr &pMap) : _pMap(pMap) {}
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
  const Map::ConstPtr &_pMap;
};

}  // namespace pathplanning

#endif
