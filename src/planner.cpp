#include "planner.h"
#include "spline.h"
#include "helpers.h"
#include <algorithm>
#include <iterator>
#include <boost/log/trivial.hpp>

namespace {

using namespace pathplanning;

tk::spline GetSplineFromPath(const Path &path) {
  std::vector<double> x;
  std::vector<double> y;
  std::tie(x, y) = ConverPathToXY(path);
  return tk::spline(x, y);
}

Eigen::Matrix3d GetTransfom(double angle, double transX, double transY) {
  Eigen::Transform<double, 2, Eigen::Isometry> T;
  T.setIdentity();
  T.linear() = Eigen::Rotation2D<double>(angle).matrix();
  T.translation() << transX, transY;
  return T.matrix();
}

Eigen::Matrix3d GetTransfom(const std::array<double, 3> &pose) {
  return GetTransfom(pose[2], pose[0], pose[1]);
}

std::array<double, 2>
Transform2D(const std::array<double, 2> &vec, const Eigen::Matrix3d &T) {
  Eigen::Vector2d transformed = Eigen::Map<const Eigen::Vector2d>(vec.data(), 2);
  transformed = (T * transformed.homogeneous()).topRows<2>();
  return {transformed[0], transformed[1]};
}

std::array<double, 2>
Transform2D(const std::array<double, 2> &vec, double angle, double transX, double transY) {
  Eigen::Matrix3d T = GetTransfom(angle, transX, transY);
  return Transform2D(vec, T);
}

std::array<double, 2>
Transform2D(const std::array<double, 2> &vec, std::array<double, 3> &pose) {
  return Transform2D(vec, pose[2], pose[0], pose[1]);
}

Path
TransformPath(const Path &path, const Eigen::Matrix3d &T) {

  Path transformedPath(path.size());

  for (size_t i = 0; i < path.size(); ++i) {
    const std::array<double, 2> &point = path[i];

    Eigen::Vector2d transformed = Eigen::Map<const Eigen::Vector2d>(point.data(), 2);
    transformed = (T * transformed.homogeneous()).topRows<2>();
    transformedPath[i] = {transformed[0], transformed[1]};

    // BOOST_LOG_TRIVIAL(debug)
    //   << "    point: x=" << point[0] << ", " << point[1]
    //   << "    transformed : x=" << transformed[0] << ", " << transformed[1];
  }
  return transformedPath;
}

}

namespace pathplanning {


Path GeneratePath(const CarState &carState) {
  // The time difference between each path point in the project is 0.02 seconds,
  // setting this distDelta will change the speed per 0.02 second.
  double distDelta = 0.5; // 0.5 meters per 0.02 seconds
  Path path;
  const std::array<double, 3> &pose = carState.euclideanPose;
  for (int i = 0; i < 50; ++i) {
    path.push_back({
      pose[0] + distDelta * i * std::cos(pose[2]),
      pose[1] + distDelta * i * std::sin(pose[2])
    });
  }
  return path;
}

Path GeneratePath(const CarState &carState, const NaviMap &naviMap) {
  // The time difference between each path point in the project is 0.02 seconds,
  // setting this distDelta will change the speed per 0.02 second.
  double distDelta = 0.5; // 0.5 
  double s = 0.0;
  const double d = 6; // 2 + 4 from the road center, 1.5 lane width

  Path path;
  for (int i = 0; i < 50; ++i) {
    s = carState.frenetPose[0] + (i + 1) * distDelta;
    std::vector<double> xy = getXY(s, d, naviMap.s, naviMap.x, naviMap.y);
    path.push_back({xy[0], xy[1]});
  }
  return path;
}


Path GeneratePath(
  int currLaneId,
  int targetLandId,
  double speedReference,
  const CarState &carState,
  const Path &prevPath,
  const NaviMap &naviMap,
  int numPreservedWaypoints
)
{
  // Create a list of widely spaced (x, y) points, evenly spaced at 30m
  // Later we will interpolate these waypoints with a spline and fill it in with
  // more points that control speed.
  // BOOST_LOG_TRIVIAL(debug)
  //   << (boost::format("Current car state: %s, speedReference=%.3f")
  //     % carState % speedReference).str();

  const double &currCarX = carState.euclideanPose[0];
  const double &currCarY = carState.euclideanPose[1];
  const double &currCarYaw = carState.euclideanPose[2];

  Path splineAnchorPoints;

  // Reference x, y, yaw states, either we will reference the starting point
  // as where the car is or at the previous paths end point
  std::array<double, 3> newTrajectoryRefenencePose;

  if (prevPath.size() < numPreservedWaypoints) {
    // If previous path is empty, use current car pose as starting reference pose
    newTrajectoryRefenencePose = carState.euclideanPose;

    // Since there is no previous trajectory left,
    // use artificial previous car state
    double prevCarX = currCarX - std::cos(currCarYaw);
    double prevCarY = currCarY - std::sin(currCarYaw);

    splineAnchorPoints.push_back({prevCarX, prevCarY});
    splineAnchorPoints.push_back({currCarX, currCarY});
  }

  else {
    // the last trajectory point from previous trajectory
    auto lastPoint = *(prevPath.end() - 1);

    // the 2nd last trajetory point from previous trajectory
    // FIXME: Use a better name
    auto lastPoint2 = *(prevPath.end() - 2);

    newTrajectoryRefenencePose = {
      lastPoint[0],
      lastPoint[1],
      // Compute the yaw angle from the last two points from previous trajectory
      std::atan2(
        lastPoint[1] - lastPoint2[1],
        lastPoint[0] - lastPoint2[0]
      )
    };

    splineAnchorPoints.push_back(lastPoint2);
    splineAnchorPoints.push_back(lastPoint);
  }

  std::vector<double> carPositionInFrenet = getFrenet(
    carState.euclideanPose[0],
    carState.euclideanPose[1],
    carState.euclideanPose[2],
    naviMap.x,
    naviMap.y
  );

  // Get 3 anchor points from the frenet
  constexpr double laneWidth = 4.0; // in meter
  constexpr double halfLaneWidth = 2.0; // in meter
  std::vector<double> targetSValueIntervalsDelta = {30, 60, 90}; // in meter
  double targetDValue = halfLaneWidth + targetLandId * laneWidth;

  for (double targetSValueDelta : targetSValueIntervalsDelta) {
    std::vector<double> xy = getXY(
      carState.frenetPose[0] + targetSValueDelta,
      targetDValue,
      naviMap.s,
      naviMap.x,
      naviMap.y
    );
    splineAnchorPoints.push_back({xy[0], xy[1]});
  }

  // NOTE: Shift the car reference angle to 0 degree
  // Transform the anchor points to car local frame
  Eigen::Matrix3d T = GetTransfom(newTrajectoryRefenencePose);
  // BOOST_LOG_TRIVIAL(debug)
  //   << "newTrajectoryRefenencePose: \n" << T;

  splineAnchorPoints = TransformPath(splineAnchorPoints, T.inverse());

  // FIXME: Spline fitting will require the points sorted in ascent order,
  // otherwise it will be error
  tk::spline spline = GetSplineFromPath(splineAnchorPoints);

  Path newPath;

  // Start with all often previous path points from last time
  for (size_t i = 0; i < prevPath.size(); ++i) {
    newPath.push_back(prevPath[i]);
  }

  // Calculate how to break up spline points so that we travel at our desired
  // reference velocity
  const double targetX = 30.0; // in meter
  const double targetY = spline(targetX);
  const double targetDist = std::sqrt(targetX * targetX + targetY * targetY);

  constexpr size_t totalNumPoints = 50;
  constexpr double timeInterval = 0.02; // seconds

  // TODO: Move
  // fill up the rest of our path planner after filling it with previous points,
  // here we will always output 50 points.

  // double numSegments = targetDist / (timeInterval * speedReference);
  double numSegments = targetDist / (timeInterval * 50 / 2.24);
  double segmentXLength = targetX / numSegments;

  // BOOST_LOG_TRIVIAL(debug)
  //   << (boost::format("numSegments=%s, segmentXLength=%s")
  //     % numSegments % segmentXLength).str();
  // BOOST_LOG_TRIVIAL(debug)
  //   << "New trajectory:";
  double x = 0.0;
  for (size_t i = 1; i <= totalNumPoints - prevPath.size(); ++i) {
    // Break the target distance into N pieces
    // interpolate the hypothenous to be a curve (instead of the straight line)
    double y = spline(x);
    x += segmentXLength;

    auto newPathPoint = Transform2D({x, y}, T);

    newPath.push_back(newPathPoint);

    // BOOST_LOG_TRIVIAL(debug)
    //   << "    x=" << newPathPoint[0]
    //   << ", y=" << newPathPoint[1];
  }

  return newPath;
}

} // namespace pathplanning
