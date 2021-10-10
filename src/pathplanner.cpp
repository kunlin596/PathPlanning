#include "pathplanner.h"
#include "spline.h"
#include "helpers.h"
#include <algorithm>
#include <iterator>

namespace {

using namespace pathplanning;

tk::spline GetSplineFromPath(const Path &path) {
  std::vector<double> x;
  std::vector<double> y;
  std::tie(x, y) = ConverPathToXY(path);
  return tk::spline(x, y);
}

Eigen::Matrix3d GetTransform(double angle, double transX, double transY) {
  // Angle should be in radian
  Eigen::Transform<double, 2, Eigen::Isometry> T;
  T.setIdentity();
  T.linear() = Eigen::Rotation2D<double>(angle).matrix();
  T.translation() << transX, transY;
  return T.matrix();
}

Eigen::Matrix3d GetTransform(const std::array<double, 3> &pose) {
  return GetTransform(pose[0], pose[1], pose[2]);
}

std::array<double, 2>
Transform2D(const std::array<double, 2> &vec, const Eigen::Matrix3d &T) {
  Eigen::Vector2d transformed = Eigen::Map<const Eigen::Vector2d>(vec.data(), 2);
  transformed = (T * transformed.homogeneous()).topRows<2>();
  return {transformed[0], transformed[1]};
}

std::array<double, 2>
Transform2D(const std::array<double, 2> &vec, double angle, double transX, double transY) {
  Eigen::Matrix3d T = GetTransform(angle, transX, transY);
  return Transform2D(vec, T);
}

std::array<double, 2>
Transform2D(const std::array<double, 2> &vec, std::array<double, 3> &pose) {
  return Transform2D(vec, pose[0], pose[1], pose[2]);
}

Path
TransformPath(const Path &path, const Eigen::Matrix3d &T) {
  Path transformedPath(path.size());

  for (size_t i = 0; i < path.size(); ++i) {
    const std::array<double, 2> &point = path[i];

    Eigen::Vector2d transformed = Eigen::Map<const Eigen::Vector2d>(point.data(), 2);
    transformed = (T * transformed.homogeneous()).topRows<2>();
    transformedPath[i] = {transformed[0], transformed[1]};
  }
  return transformedPath;
}


std::array<double, 3> GetEndPose(const Path &path) {
  auto point1 = path[path.size() - 2];
  auto point2 = path[path.size() - 1];
  return {
    std::atan2(
      point2[1] - point1[1],
      point2[0] - point1[0]
    ),
    point2[0],
    point2[1]
  };
}

}

namespace pathplanning {

SensorFusions PathPlanner::_Filter(const SensorFusions &sensorFusions, double radius)
{
  SensorFusions filtered;
  const std::array<double, 2> egoLocation = {
    _carState.euclideanPose[1],
    _carState.euclideanPose[2]
  };

  // TODO:
  for (size_t i = 0; i < sensorFusions.size(); ++i) {
    // Sensed info for each car
  }

  return sensorFusions;
}

Goal GoalGenerator::_GenerateLKGoal(
  const double speedReference,
  const SensorFusions &sensorFusions,
  const CarState &carState,
  const Path &prevPath) {

  static constexpr double distanceToKeep = 30; // in meter
  static constexpr double timeInterval = 0.02;
  static constexpr double speedLimit = 49.5; // MPH

  double targetSpeed = speedReference;
  // Target speed is based on the previous behavior speed, not the current car speed.
  int targetLaneId = carState.lane.id;

  const double currSValue = carState.frenetPose[0];
  const double currDValue = carState.frenetPose[1];

  // Expected S value of the car if the previous trajectory are executed
  double expectedSValue = currSValue;
  if (prevPath.size() > 0) {
    std::array<double, 3> pose = GetEndPose(prevPath);
    std::vector<double> frenetPose = getFrenet(
      pose[1], pose[2], pose[0],
      _map.x, _map.y);

    expectedSValue = frenetPose[0];
  }

  double closestCarSpeed = std::numeric_limits<double>::quiet_NaN();
  double closestCarSValue = std::numeric_limits<double>::max();

  bool tooClose = false;

  for (size_t i = 0; i < sensorFusions.size(); ++i) {
    // Sensed info for each car
    const double &speed = sensorFusions[i].speed;
    // Predict where the car will be in the future
    const double s = sensorFusions[i].frenetPose[0] + static_cast<double>(prevPath.size()) * timeInterval * speed / 2.24;
    const double &d = sensorFusions[i].frenetPose[1];
    // If the the other car is in the same lane
    // TODO: Replace the range with continous d values to prevent accident
    if ((_map.road.GetLaneCenterDValue(targetLaneId - 1) < d) and d < (_map.road.GetLaneCenterDValue(targetLaneId + 1)) ) {

      double distance = s - expectedSValue;

      if (distance > 0 and distance < distanceToKeep) {
        if (s < closestCarSValue) {
          closestCarSValue = s;
          closestCarSpeed = speed;
        }
      }
    }
  }

  static constexpr double acc = 0.224;  // ~5 meters / second^2

  if (!std::isnan(closestCarSpeed)) {
    targetSpeed -= acc;
  } else if (targetSpeed < speedLimit) {
    targetSpeed += acc;
  }

  targetSpeed = std::min(targetSpeed, speedLimit); // Speed limit

  return Goal(targetSpeed, targetLaneId);


}

Goal GoalGenerator::_GenerateLCPGoal(
  const SensorFusions &sensorFusions,
  const CarState &carState,
  const Path &prevPath)
{
  return Goal(1.0, 1);
}

Goal GoalGenerator::_GenerateLCGoal(
  const SensorFusions &sensorFusions,
  const CarState &carState,
  const Path &prevPath)
{
  return Goal(1.0, 1);
}

Goal GoalGenerator::GenerateGoal(
  const BehaviorState &behaviorState,
  const double speedReference,
  const SensorFusions &sensorFusions,
  const CarState &carState,
  const Path &prevPath)
{
  switch (behaviorState) {
    case BehaviorState::kLaneKeeping:
      return _GenerateLKGoal(
        speedReference, sensorFusions, carState, prevPath);
  }
}

Path GeneratePath(const CarState &carState) {
  // The time difference between each path point in the project is 0.02 seconds,
  // setting this distDelta will change the speed per 0.02 second.
  double distDelta = 0.5; // 0.5 meters per 0.02 seconds
  Path path;
  const std::array<double, 3> &pose = carState.euclideanPose;
  for (int i = 0; i < 50; ++i) {
    path.push_back({
      pose[1] + distDelta * i * std::cos(pose[0]),
      pose[2] + distDelta * i * std::sin(pose[0])
    });
  }
  return path;
}


Path GeneratePath(const CarState &carState, const NaviMap &naviMap) {
  // The time difference between each path point in the project is 0.02 seconds,
  // setting this distDelta will change the speed per 0.02 second.
  double distDelta = 0.5;
  double s = 0.0;

  static constexpr double d = 6.0; // 2 + 4 from the road center, 1.5 lane width
  static constexpr int totalNumPoints = 50;

  Path path;
  for (int i = 0; i < totalNumPoints; ++i) {
    s = carState.frenetPose[0] + (i + 1) * distDelta;
    std::vector<double> xy = getXY(s, d, naviMap.s, naviMap.x, naviMap.y);
    path.push_back({xy[0], xy[1]});
  }

  return path;
}


/**
 * Generate a spline fitting in vehicle's local frame
 */
Path PathPlanner::_GeneratePath(
  const double targetDValue,
  const double targetSValue,
  const double speedReference
)
{
  // cout << boost::format("_GeneratePath: targetDValue=%.3f, targetSValue=%.3f, speedReference=%.3f\n")
  //   % targetDValue % targetSValue % speedReference;

  const double &currCarYaw = _carState.euclideanPose[0];
  const double &currCarX = _carState.euclideanPose[1];
  const double &currCarY = _carState.euclideanPose[2];

  Path splineAnchorPoints;

  // Reference x, y, yaw states, either we will reference the starting point
  // as where the car is or at the previous paths end point
  std::array<double, 3> newTrajectoryRefenencePose;

  // If previous path is empty, use current car pose as starting reference pose
  newTrajectoryRefenencePose = _carState.euclideanPose;
  if (_prevPath.size() < 1) {
    // Since there is no previous trajectory left,
    // use artificial previous car state
    double prevCarX = currCarX - std::cos(currCarYaw);  // in radians
    double prevCarY = currCarY - std::sin(currCarYaw);  // in radians
    splineAnchorPoints.push_back({prevCarX, prevCarY});
    splineAnchorPoints.push_back({currCarX, currCarY});
  } else {
    auto point1 = _prevPath[_prevPath.size() - 2];
    auto point2 = _prevPath[_prevPath.size() - 1];
    splineAnchorPoints.push_back(point1);
    splineAnchorPoints.push_back(point2);
    newTrajectoryRefenencePose = {
      std::atan2(
        point2[1] - point1[1],
        point2[0] - point1[0]
      ),
      point2[0],
      point2[1]
    };
  }

  // PrintPath(splineAnchorPoints);

  constexpr int numAnchorPoints = 5;
  const double deltaSValue = targetSValue / static_cast<double>(numAnchorPoints);
  std::vector<double> targetSValueDeltas(numAnchorPoints); // in meter
  for (int i = 0; i < numAnchorPoints; ++i) {
    targetSValueDeltas[i] = deltaSValue * (i + 1);
  }

  // expectedEndS is for attaching the newly planned points to the previous path
  std::vector<double> expectedFrenetPoseEndPath = getFrenet(
    newTrajectoryRefenencePose[1],
    newTrajectoryRefenencePose[2],
    newTrajectoryRefenencePose[0],
    _map.x, _map.y);

  for (double targetSValueDelta : targetSValueDeltas) {
    std::vector<double> loc = getXY(
      expectedFrenetPoseEndPath[0] + targetSValueDelta, targetDValue,
      _map.s, _map.x, _map.y);
    splineAnchorPoints.push_back({loc[0], loc[1]});
  }

  // PrintPath(splineAnchorPoints);

  // NOTE: Shift the car reference angle to 0 degree
  // Transform the anchor points to car local frame
  Eigen::Matrix3d T = GetTransform(newTrajectoryRefenencePose);

  // PrintPath(splineAnchorPoints);
  splineAnchorPoints = TransformPath(splineAnchorPoints, T.inverse());

  // FIXME: Spline fitting will require the points sorted in ascent order,
  // otherwise it will be error
  // std::sort(
  //   splineAnchorPoints.begin(), splineAnchorPoints.end(),
  //   [] (const std::array<double, 2> &p1, const std::array<double, 2> &p2) {
  //     return p1[0] < p2[0];
  //   }
  // );
  tk::spline splineFn = GetSplineFromPath(splineAnchorPoints);

  Path newPath;

  // Start with all often previous path points from last time
  if (_prevPath.size() > 0) {
    std::copy(_prevPath.begin(), _prevPath.end(), std::back_inserter(newPath));
  }

  // Calculate how to break up spline points so that we travel at our desired
  // reference velocity
  const double targetX = targetSValue;
  const double targetY = splineFn(targetX);
  const double targetDist = std::sqrt(targetX * targetX + targetY * targetY);

  static constexpr size_t totalNumPoints = 50;
  static constexpr double timeInterval = 0.02; // seconds

  // TODO: Move
  // fill up the rest of our path planner after filling it with previous points,
  // here we will always output 50 points.

  // in meter, 1/2.24 is converting MPH to meters/sec
  const double numSegments = targetDist / (timeInterval * speedReference / 2.24);
  const double segmentXLength = targetX / numSegments;

  double x = 0.0;
  double y = 0.0;
  const size_t numPointsNeedsToBeGenerated = totalNumPoints - _prevPath.size();

  // cout << "Generating numPointsNeedsToBeGenerated=" << numPointsNeedsToBeGenerated << endl;
  for (size_t i = 0; i < numPointsNeedsToBeGenerated; ++i) {
    // Break the target distance into N pieces
    // interpolate the hypothenous to be a curve (instead of the straight line)

    x = segmentXLength * (i + 1);
    y = splineFn(x);

    // Transform the points back to global frame
    const std::array<double, 2> newPathPoint = Transform2D({x, y}, T);

    newPath.push_back(newPathPoint);
  }

  // PrintPath(newPath);

  return newPath;
}

int _GetLaneId(int currLaneId, const std::string &direction) {
  // TODO: Add more checks
  if (direction == "left") {
    return currLaneId - 1;
  } else if (direction == "right") {
    return currLaneId + 1;
  }
  return -1;
}

Path PathPlanner::GeneratePath(const Goal &goal, const Behavior &currBehavior)
{
  double targetSpeed = goal.speed;
  int targetLaneId = currBehavior.lane.id;

  double targetDValue = _map.road.GetLaneCenterDValue(currBehavior.lane.id);
  double targetSValue = 90.0;

  return _GeneratePath(targetDValue, targetSValue, targetSpeed);
}

} // namespace pathplanning
