#ifndef PATHPLANNING_JMT_H
#define PATHPLANNING_JMT_H

#include <array>
#include <unordered_map>

#include "math.h"
#include "path.h"
#include "vehicle.h"

namespace pathplanning {

/**
 * @brief      This class describes a sd waypoint function.
 */
class SDFunctor {
 public:
  SDFunctor() {}

  SDFunctor(const QuinticFunctor& sFunc, const QuinticFunctor& dFunc)
      : _sFunc(sFunc), _dFunc(dFunc) {}

  std::array<double, 2> Eval(const double x) const {
    return {_sFunc(x), _dFunc(x)};
  }

  std::array<double, 2> operator()(const double x) { return Eval(x); }

  const QuinticFunctor& GetSFunc() const { return _sFunc; }
  const QuinticFunctor& GetDFunc() const { return _dFunc; }

 private:
  QuinticFunctor _sFunc;
  QuinticFunctor _dFunc;
};

/**
 * @brief      JMT Trajectory representation
 *
 * JMT trajectory is used for producing a waypoint at a given time.
 *
 * This trajectory is also flexible for evaluation since it preserves a lot of
 * useful information than the final product which is a set of discrete
 * waypoints.
 *
 * Note that not all trajectory generators use the same information to produce
 * the same format of output, so trajecotry is per generator type.
 */
struct JMTTrajectory {
  SDFunctor sdFunc;
  double elapsedTime = 0.0;

  JMTTrajectory() {}

  JMTTrajectory(const SDFunctor& sdFunc, const double elapsedTime)
      : sdFunc(sdFunc), elapsedTime(elapsedTime) {}

  inline Waypoint Eval(const double t) { return sdFunc(t); }

  Waypoint operator()(const double t) { return Eval(t); }
};

/**
 * @brief      JMT generator
 *
 * This class minimizes a 5th order polynomial representing the coordinate
 * w.r.t. time.
 *
 * The 6 coefficients below are the ones we are solving. Giving that we are
 * minimizing the jerk.
 *
 * s(t) =
 *    a0 +
 *    a1 * t +
 *    a2 * t^2 +
 *    a3 * t^3 +
 *    a4 * t^4 +
 *    a5 * t^5
 *
 * By choosing t_i = 0, 6 equations becomes 3 equations.
 *
 * s (t_i) = s (0) = a0
 * s'(t_i) = s'(0) = a1
 * s"(t_i) = s"(0) = 2 * a2
 *
 * s (t) = a3 * t^3 + a4 * t^4 + a5 * t^5                 + C1
 * s'(t) = 3 * a3 * t^2 + 4 * a4 * t^3 + 5 * a5 * t^4     + C2
 * s"(t) = 6 * a3 * t^2 + 12 * a4 * t^2 + 20 * a5 * t^3   + C3
 *
 * Set t = t_f, t_f is the end time stamp.
 *
 * s (t_f) = a3 * t_f^3 + a4 * t_f^4 + a5 * t_f^5                 + C1
 * s'(t_f) = 3 * a3 * t_f^2 + 4 * a4 * t_f^3 + 5 * a5 * t_f^4     + C2
 * s"(t_f) = 6 * a3 * t_f^2 + 12 * a4 * t_f^2 + 20 * a5 * t_f^3   + C3
 *
 * s(t_f), s'(t_f), s"(t_f) and t_f are the boundary conditions set by the user,
 * so they are known quantities.
 *
 * So it becomes a linear system with 3 equations and 3 unknowns.
 *
 * Ax = b
 *
 * where,
 * A = [
 *  [t_f^3,     t_f^4,      t_f^5     ],
 *  [3 * t_f^2, 4 * t_f^3,  5 * t_f^4 ],
 *  [6 * t_f,   12 * t_f^2, 20 * t_f^3],
 * ]
 *
 * x = [a3, a4, a5]
 *
 * b = [s(t_f), s'(t_f), s"(t_f)]
 *
 * The solution above is for 1D JMT for s, the same logic applies to d.
 *
 * Combine these 2 resulrs, we get a real 2D drivable trajectory.
 *
 */
struct JMT {
  static QuinticFunctor Solve1D(const std::array<double, 6>& params,
                                const double t);

  /**
   * @brief      Compute a SDFunctor
   *
   * S paramerers, [s(i), s'(i), s"(i), s(f), s'(f), * s"(f)]
   * D parameters, [d(i), d'(i), d"(i), d(f), d'(f), * d"(f)]
   *
   * @param[in]  sParams     The s parameters,
   * @param[in]  dParams     The d parameters
   * @param[in]  t           The target time to reach the goal
   *
   * @return     The sd waypoint function.
   */
  static SDFunctor Solve2D(const std::array<double, 6>& sParams,
                           const std::array<double, 6>& dParams,
                           const double t);

  static JMTTrajectory ComputeTrajectory(const std::array<double, 6>& sParams,
                                         const std::array<double, 6>& dParams,
                                         const double t);

  static JMTTrajectory ComputeTrajectory(const VehicleConfiguration& start,
                                         const VehicleConfiguration& end,
                                         const double t);
};

/* -------------------------------------------------------------------------- */
// Trajectory evaluation
namespace costs {
enum class CostType {
  kTimeDiff,
  kSDiff,
  kDDiff,
  kCollision,
  kBuffer,
  kStaysOnRoad,
  kExceedsSpeedLimit,
  kEfficiency,
  kTotalAccel,
  kMaxAccel,
  kTotalJerk,
  kMaxJerk
};

using CostWeightMapping = std::unordered_map<CostType, double>;

struct CostFunctor {
  virtual double Compute(
      const JMTTrajectory& traj, const int targetVehicleId, const double delta,
      const double time,
      const std::unordered_map<int, Vehicle>& perceptions) = 0;
  virtual ~CostFunctor() {}
};

struct TimeDiffCost : public CostFunctor {
  double Compute(const JMTTrajectory& traj, const int targetVehicleId,
                 const double delta, const double time,
                 const std::unordered_map<int, Vehicle>& perceptions) override {
    return Logistic(static_cast<double>(std::abs(traj.elapsedTime - time)) /
                    time);
  }
};

struct SDiffCost : public CostFunctor {
  double Compute(const JMTTrajectory& traj, const int targetVehicleId,
                 const double delta, const double time,
                 const std::unordered_map<int, Vehicle>& perceptions) override {
    // TODO
    return 0.0;
  }
};

struct DDiffCost : public CostFunctor {
  double Compute(const JMTTrajectory& traj, const int targetVehicleId,
                 const double delta, const double time,
                 const std::unordered_map<int, Vehicle>& perceptions) override {
    // TODO
    return 0.0;
  }
};

struct CollisionCost : public CostFunctor {
  double Compute(const JMTTrajectory& traj, const int targetVehicleId,
                 const double delta, const double time,
                 const std::unordered_map<int, Vehicle>& perceptions) override {
    // TODO
    return 0.0;
  }
};

struct BufferCost : public CostFunctor {
  double Compute(const JMTTrajectory& traj, const int targetVehicleId,
                 const double delta, const double time,
                 const std::unordered_map<int, Vehicle>& perceptions) override {
    // TODO
    return 0.0;
  }
};

struct StaysOnRoadCost : public CostFunctor {
  double Compute(const JMTTrajectory& traj, const int targetVehicleId,
                 const double delta, const double time,
                 const std::unordered_map<int, Vehicle>& perceptions) override {
    // TODO
    return 0.0;
  }
};

struct ExceedsSpeedLimitCost : public CostFunctor {
  double Compute(const JMTTrajectory& traj, const int targetVehicleId,
                 const double delta, const double time,
                 const std::unordered_map<int, Vehicle>& perceptions) override {
    // TODO
    return 0.0;
  }
};

struct EfficiencyCost : public CostFunctor {
  double Compute(const JMTTrajectory& traj, const int targetVehicleId,
                 const double delta, const double time,
                 const std::unordered_map<int, Vehicle>& perceptions) override {
    // TODO
    return 0.0;
  }
};

struct TotalAccelCost : public CostFunctor {
  double Compute(const JMTTrajectory& traj, const int targetVehicleId,
                 const double delta, const double time,
                 const std::unordered_map<int, Vehicle>& perceptions) override {
    // TODO
    return 0.0;
  }
};

struct MaxAccelCost : public CostFunctor {
  double Compute(const JMTTrajectory& traj, const int targetVehicleId,
                 const double delta, const double time,
                 const std::unordered_map<int, Vehicle>& perceptions) override {
    // TODO
    return 0.0;
  }
};
struct TotalJerkCost : public CostFunctor {
  double Compute(const JMTTrajectory& traj, const int targetVehicleId,
                 const double delta, const double time,
                 const std::unordered_map<int, Vehicle>& perceptions) override {
    // TODO
    return 0.0;
  }
};
struct MaxJerkCost : public CostFunctor {
  double Compute(const JMTTrajectory& traj, const int targetVehicleId,
                 const double delta, const double time,
                 const std::unordered_map<int, Vehicle>& perceptions) override {
    // TODO
    return 0.0;
  }
};

std::shared_ptr<CostFunctor> CreateCostFunctor(const CostType& type);

}  // namespace costs

class JMTTrajectoryValidator {
 public:
  JMTTrajectoryValidator(const costs::CostWeightMapping& costWeightMapping)
      : _costWeightMapping(costWeightMapping) {}

  double Validate(const JMTTrajectory& traj, const int targetVehicleId,
                  const double delta, const double time,
                  const std::unordered_map<int, Vehicle>& perceptions) {
    double cost = 0.0;
    for (const auto& costInfo : _costWeightMapping) {
      if (_funcPtrs.count(costInfo.first) == 0) {
        _funcPtrs[costInfo.first] = costs::CreateCostFunctor(costInfo.first);
      }
      cost += _funcPtrs[costInfo.first]->Compute(traj, targetVehicleId, delta,
                                                 time, perceptions) *
              costInfo.second;
    }
    return cost;
  }

 private:
  costs::CostWeightMapping _costWeightMapping;
  std::unordered_map<costs::CostType, std::shared_ptr<costs::CostFunctor>>
      _funcPtrs;
};

}  // namespace pathplanning

#endif
