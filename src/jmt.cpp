#include "jmt.h"

#include "Eigen-3.3/Eigen/Dense"

namespace pathplanning {

namespace costs {
std::shared_ptr<CostFunctor> CreateCostFunctor(const CostType &type) {
  switch (type) {
    case CostType::kTimeDiff:
      return std::make_shared<TimeDiffCost>();
    case CostType::kSDiff:
      return std::make_shared<SDiffCost>();
    case CostType::kDDiff:
      return std::make_shared<DDiffCost>();
    case CostType::kCollision:
      return std::make_shared<CollisionCost>();
    case CostType::kBuffer:
      return std::make_shared<BufferCost>();
    case CostType::kStaysOnRoad:
      return std::make_shared<StaysOnRoadCost>();
    case CostType::kExceedsSpeedLimit:
      return std::make_shared<ExceedsSpeedLimitCost>();
    case CostType::kEfficiency:
      return std::make_shared<EfficiencyCost>();
    case CostType::kTotalAccel:
      return std::make_shared<TotalAccelCost>();
    case CostType::kMaxAccel:
      return std::make_shared<MaxAccelCost>();
    case CostType::kTotalJerk:
      return std::make_shared<TotalJerkCost>();
    case CostType::kMaxJerk:
      return std::make_shared<MaxJerkCost>();
    default:
      throw std::runtime_error("Not supported cost function");
  }
}
}  // namespace costs

QuinticFunctor JMT::Solve1D(const std::array<double, 3> &start,
                            const std::array<double, 3> &end, const double t) {
  std::array<double, 6> coeffs;
  Eigen::Matrix3d A;
  const double t2 = t * t;
  const double t3 = t2 * t;
  const double t4 = t3 * t;
  const double t5 = t4 * t;
  // clang-format off
  A <<
    t3     , t4      , t5      ,
    3 * t2 , 4 * t3  , 5 * t4  ,
    6 * t  , 12 * t2 , 20 * t3 ;

  Eigen::Vector3d b = {
      end[0] - (start[0] + start[1] * t + start[2] * t2 / 2.0),
      end[1] - (start[1] + start[2] * t),
      end[2] - start[2]
  };
  // clang-format on

  // See
  // https://eigen.tuxfamily.org/dox-devel/group__TutorialLinearAlgebra.html
  Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);
  coeffs[0] = start[0];
  coeffs[1] = start[1];
  coeffs[2] = start[2] / 2.0;
  coeffs[3] = x[0];
  coeffs[4] = x[1];
  coeffs[5] = x[2];
  // SPDLOG_INFO(params);
  return QuinticFunctor(coeffs);
}

QuinticFunctor JMT::Solve1D(const std::array<double, 6> &params,
                            const double t) {
  return Solve1D({params[0], params[1], params[2]},
                 {params[3], params[4], params[5]}, t);
}

SDFunctor JMT::Solve2D(const std::array<double, 6> &sParams,
                       const std::array<double, 6> &dParams, const double t) {
  return SDFunctor(JMT::Solve1D(sParams, t), JMT::Solve1D(dParams, t));
}

JMTTrajectory JMT::ComputeTrajectory(const std::array<double, 6> &sParams,
                                     const std::array<double, 6> &dParams,
                                     const double t) {
  return JMTTrajectory(JMT::Solve2D(sParams, dParams, t), t);
}

JMTTrajectory JMT::ComputeTrajectory(const VehicleConfiguration &start,
                                     const VehicleConfiguration &end,
                                     const double t) {
  // clang-format off
  return JMT::ComputeTrajectory(
      // sParams
      std::array<double, 6>{
        start.At(0), start.At(1), start.At(2),
        end.At(0)  , end.At(1)  , end.At(2)
      },
      // dParams
      std::array<double, 6>{
        start.At(3), start.At(4), start.At(5),
        end.At(3)  , end.At(4)  , end.At(5)
      },
      t
    );
  // clang-format on
}

double JMTTrajectoryEvaluator::Validate(const JMTTrajectory &traj,
                                        const double time,
                                        const Predictions &predictions) {
  double cost = 0.0;
  for (const auto &costInfo : _costWeightMapping) {
    if (_funcPtrs.count(costInfo.first) == 0) {
      _funcPtrs[costInfo.first] = costs::CreateCostFunctor(costInfo.first);
    }
    cost += _funcPtrs[costInfo.first]->Compute(traj, time, predictions) *
            costInfo.second;
  }
  return cost;
}

}  // namespace pathplanning
