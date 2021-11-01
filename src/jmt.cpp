#include "jmt.h"

#include "Eigen-3.3/Eigen/Dense"
#include "traj_evaluator.h"

namespace pathplanning {

JMTTrajectory1D
JMT::Solve1D(const std::array<double, 3>& start,
             const std::array<double, 3>& end,
             const double t)
{
  std::array<double, 6> coeffs;
  Eigen::Matrix3d A;
  double t2 = t * t;
  double t3 = t2 * t;
  double t4 = t3 * t;
  double t5 = t4 * t;
  // clang-format off
  A <<
    t3       , t4        , t5        ,
    3.0 * t2 , 4.0 * t3  , 5.0 * t4  ,
    6.0 * t  , 12.0 * t2 , 20.0 * t3 ;

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
  SPDLOG_TRACE(coeffs);
  return JMTTrajectory1D(coeffs, t);
}

JMTTrajectory1D
JMT::Solve1D(const std::array<double, 6>& params, const double t)
{
  return Solve1D({ params[0], params[1], params[2] },
                 { params[3], params[4], params[5] },
                 t);
}

JMTTrajectory2D
JMT::Solve2D(const std::array<double, 6>& sParams,
             const std::array<double, 6>& dParams,
             const double t)
{
  return { JMT::Solve1D(sParams, t), JMT::Solve1D(dParams, t) };
}

JMTTrajectory2D
JMT::ComputeTrajectory(const std::array<double, 6>& sParams,
                       const std::array<double, 6>& dParams,
                       const double t)
{
  return JMT::Solve2D(sParams, dParams, t);
}

JMTTrajectory2D
JMT::ComputeTrajectory(const VehicleConfiguration& start,
                       const VehicleConfiguration& end,
                       const double t)
{
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

bool
JMTTrajectory1D::IsValid(const ValidationParams& params) const
{
  double currtime = 0.0;
  while (currtime < _time + 1e-6) {
    auto values = Eval(currtime);
    SPDLOG_INFO("{}", values);
    if (params.speedRange[0] > values[1] or values[1] > params.speedRange[1]) {
      SPDLOG_ERROR("speed limit: {}", values);
      return false;
    }
    if (params.accelRange[0] > values[2] or values[2] > params.accelRange[1]) {
      SPDLOG_ERROR("accel limit: {}", values);
      return false;
    }
    currtime += params.timeResolution;
  }
  return true;
}

double
JMTTrajectory2D::ComputeNearestApproach(const Vehicle& vehicle,
                                        double maxTimeDuration,
                                        double timeStep) const
{
  double minDist = std::numeric_limits<double>::infinity();
  double currTime = 0.0;
  while (currTime < maxTimeDuration) {
    currTime += timeStep;
    VehicleConfiguration trajConf = Eval(currTime);
    VehicleConfiguration vehicleConf = vehicle.GetConfiguration(currTime);
    double dist = GetDistance({ trajConf.sPos, trajConf.dPos },
                              { vehicleConf.sPos, vehicleConf.dPos });
    if (dist < minDist) {
      minDist = dist;
    }
  }
  return minDist;
}

double
JMTTrajectory2D::ComputeNearestApproach(const std::vector<Vehicle>& vehicles,
                                        double maxTimeDuration,
                                        double timeStep) const
{
  double minDist = std::numeric_limits<double>::infinity();
  double currTime = 0.0;

  while (currTime < maxTimeDuration) {
    currTime += timeStep;
    VehicleConfiguration trajConf = Eval(currTime);
    for (const auto& vehicle : vehicles) {
      VehicleConfiguration vehicleConf = vehicle.GetConfiguration(currTime);
      double dist = GetDistance({ trajConf.sPos, trajConf.dPos },
                                { vehicleConf.sPos, vehicleConf.dPos });
      if (dist < minDist) {
        minDist = dist;
      }
    }
  }
  return minDist;
}

double
JMTTrajectory2D::ComputeNearestApproach(
  const std::unordered_map<int, Vehicle>& vehicles,
  double maxTimeDuration,
  double timeStep) const
{
  double minDist = std::numeric_limits<double>::infinity();
  double currTime = 0.0;

  while (currTime < maxTimeDuration) {
    currTime += timeStep;
    VehicleConfiguration trajConf = Eval(currTime);
    for (const auto& vehicle : vehicles) {
      VehicleConfiguration vehicleConf =
        vehicle.second.GetConfiguration(currTime);
      double dist = GetDistance({ trajConf.sPos, trajConf.dPos },
                                { vehicleConf.sPos, vehicleConf.dPos });
      if (dist < minDist) {
        minDist = dist;
      }
    }
  }
  return minDist;
}

bool
JMTTrajectory2D::IsValid(const ValidationParams& params) const
{
  if (not _traj1.IsValid(params) or not _traj2.IsValid(params)) {
    return false;
  }
  return true;
}

std::ostream&
operator<<(std::ostream& out, const pathplanning::JMTTrajectory2D& traj)
{
  return out << fmt::format("JMTTrajectory2D(time={}, sCoeffs={}, dCoeffs{})",
                            traj.GetTime(),
                            traj.GetSFunc(),
                            traj.GetDFunc());
}

} // namespace pathplanning
