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
      pose[1] + distDelta * i * std::cos(pose[0]),
      pose[2] + distDelta * i * std::sin(pose[0])
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

double ComputeTargetSpeed(
  const Behavior &currBehavior,
  const CarState &currCarState,
  const Path &prevPath,
  const NaviMap &naviMap,
  const SensorFusions &sensorFusions,
  const std::array<double, 2> &endPathFrenetPose,
  const int targetLaneId)
{
  constexpr double distanceToKeep = 30; // in meter
  constexpr double timeInterval = 0.02;
  constexpr double speedLimit = 49.5; // MPH

  // Target speed is based on the previous behavior speed, not the current car speed.
  double targetSpeed = currBehavior.targetSpeed;

  const double currS = currCarState.frenetPose[0];
  const double currD = currCarState.frenetPose[1];

  // Expected S value of the car if the previous trajectory are executed
  double expectedS = currS;

  if (prevPath.size() > 0) {
    expectedS = endPathFrenetPose[0];
  }

  double closestCarSpeed = std::numeric_limits<double>::quiet_NaN();
  double closestCarS = std::numeric_limits<double>::max();

  bool tooClose = false;

  for (size_t i = 0; i < sensorFusions.size(); ++i) {
    // Sensed info for each car
    const double &speed = sensorFusions[i].speed;
    // Predict where the car will be in the future
    const double s = sensorFusions[i].frenetPose[0] + static_cast<double>(prevPath.size()) * timeInterval * speed / 2.24;
    const double &d = sensorFusions[i].frenetPose[1];
    // If the the other car is in the same lane
    // TODO: Replace the range with continous d values to prevent accident
    if ((naviMap.road.GetLaneCenterDValue(targetLaneId - 1) < d) and
        d < (naviMap.road.GetLaneCenterDValue(targetLaneId + 1))) {

      double distance = s - expectedS;

      if (distance > 0 and distance < 10) {
        targetSpeed = speed;
        return targetSpeed;
      }

      if (distance > 0 and distance < distanceToKeep) {
        if (s < closestCarS) {
          closestCarS = s;
          closestCarSpeed = speed;
        }
      }
    }
  }

  constexpr double acc = 0.224;  // ~5 meters / second^2
  if (!std::isnan(closestCarSpeed)) {
    targetSpeed -= acc;
  } else if (targetSpeed < speedLimit) {
    targetSpeed += acc;
  }

  return std::min(targetSpeed, speedLimit); // Speed limit
}

/**
 * Generate a spline fitting in vehicle's local frame
 */
Path _GeneratePath(
  const CarState &carState,
  const NaviMap &naviMap,
  const std::array<double, 2> &endPathFrenetPose,
  const Path &prevPath,
  const double targetDValue,
  const double targetSValue,
  const double speedReference
)
{

  const double &currCarYaw = carState.euclideanPose[0];
  const double &currCarX = carState.euclideanPose[1];
  const double &currCarY = carState.euclideanPose[2];

  Path splineAnchorPoints;

  // Reference x, y, yaw states, either we will reference the starting point
  // as where the car is or at the previous paths end point
  std::array<double, 3> newTrajectoryRefenencePose;

  // If previous path is empty, use current car pose as starting reference pose
  newTrajectoryRefenencePose = carState.euclideanPose;

  // Since there is no previous trajectory left,
  // use artificial previous car state
  double prevCarX = currCarX - std::cos(currCarYaw);  // in radians
  double prevCarY = currCarY - std::sin(currCarYaw);  // in radians

  splineAnchorPoints.push_back({prevCarX, prevCarY});
  splineAnchorPoints.push_back({currCarX, currCarY});

  const std::vector<double> carPositionInFrenet = getFrenet(
    currCarX,
    currCarY,
    currCarYaw,
    naviMap.x,
    naviMap.y
  );

  constexpr int numAnchorPoints = 5;
  const double deltaS = targetSValue / static_cast<double>(numAnchorPoints);
  std::vector<double> targetSValueDeltas(numAnchorPoints); // in meter
  for (int i = 0; i < numAnchorPoints; ++i) {
    targetSValueDeltas.push_back(deltaS * (i + 1));
  }

  // expectedEndS is for attaching the newly planned points to the previous path
  double expectedEndS = carState.frenetPose[0];
  if (prevPath.size() > 0) {
    expectedEndS = endPathFrenetPose[0];
  }

  for (double targetSValueDelta : targetSValueDeltas) {
    std::vector<double> xy = getXY(
      expectedEndS + targetSValueDelta,
      targetDValue,
      naviMap.s,
      naviMap.x,
      naviMap.y
    );
    splineAnchorPoints.push_back({xy[0], xy[1]});
  }

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
  if (prevPath.size() > 0) {
    std::copy(prevPath.begin(), prevPath.end(), std::back_inserter(newPath));
  }

  // Calculate how to break up spline points so that we travel at our desired
  // reference velocity
  const double targetX = targetSValue;
  const double targetY = splineFn(targetX);
  const double targetDist = std::sqrt(targetX * targetX + targetY * targetY);

  constexpr size_t totalNumPoints = 50;
  constexpr double timeInterval = 0.02; // seconds

  // TODO: Move
  // fill up the rest of our path planner after filling it with previous points,
  // here we will always output 50 points.

  // in meter, 1/2.24 is converting MPH to meters/sec
  const double numSegments = targetDist / (timeInterval * speedReference / 2.24);
  const double segmentXLength = targetX / numSegments;

  double x = 0.0;
  double y = 0.0;
  const size_t numPointsNeedsToBeGenerated = totalNumPoints - prevPath.size();

  for (size_t i = 0; i < numPointsNeedsToBeGenerated; ++i) {
    // Break the target distance into N pieces
    // interpolate the hypothenous to be a curve (instead of the straight line)

    x = segmentXLength * (i + 1);
    y = splineFn(x);

    // Transform the points back to global frame
    const std::array<double, 2> newPathPoint = Transform2D({x, y}, T);

    newPath.push_back(newPathPoint);
  }
  return newPath;
}

Path _GenerateLaneChangingPath(
  int targetLaneId,
  const Behavior &currBehavior,
  const CarState &carState,
  const Path &prevPath,
  const NaviMap &naviMap,
  const std::array<double, 2> &endPathFrenetPose,
  int numPreservedWaypoints
)
{
  // Generate lane changing path
  const double changingDirection = targetLaneId - currBehavior.lane.id;  // changeingDirection will be one of [-1, 0, 1]
  const double targetDValue = changingDirection * 2.0; // half of lane width
  const double targetSValue = 30; // meters
  return _GeneratePath(
    carState,
    naviMap,
    endPathFrenetPose,
    prevPath,
    targetDValue,
    targetSValue,
    currBehavior.targetSpeed
  );
}

Path _GenerateLaneKeepingPath(
  const Behavior &currBehavior,
  const double targetSpeed,
  const CarState &carState,
  const Path &prevPath,
  const NaviMap &naviMap,
  const std::array<double, 2> &endPathFrenetPose,
  int numPreservedWaypoints
)
{
  const double targetSValue = 90; // meters
  const double targetLaneId = naviMap.road.GetLandId(carState.frenetPose[1]);
  const double targetDValue = naviMap.road.GetLaneCenterDValue(targetLaneId);

  return _GeneratePath(
    carState,
    naviMap,
    endPathFrenetPose,
    prevPath,
    targetDValue,
    targetSValue,
    targetSpeed
  );
}

Path _GenerateLaneChangingPreparationPath(
  int targetLaneId,
  const Behavior &currBehavior,
  const CarState &carState,
  const Path &prevPath,
  const NaviMap &naviMap,
  const std::array<double, 2> &endPathFrenetPose,
  int numPreservedWaypoints
)
{
  const double targetSValue = 90; // meters
  const double changingDirection = targetLaneId - currBehavior.lane.id;  // changeingDirection will be one of [-1, 0, 1]
  const double dValueDelta = 0.5; // meters, offset the car to be closer to the target lane
  const double targetDValue
    = naviMap.road.GetLaneCenterDValue(targetLaneId) + changingDirection * dValueDelta;
  return _GeneratePath(
    carState,
    naviMap,
    endPathFrenetPose,
    prevPath,
    targetDValue,
    targetSValue,
    currBehavior.targetSpeed
  );
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


Path GeneratePath(
  const BehaviorState nextBehaviorState,
  const double targetSpeed,
  const Behavior &currBehavior,
  const CarState &carState,
  const Path &prevPath,
  const NaviMap &naviMap,
  const std::array<double, 2> &endPathFrenetPose,
  int numPreservedWaypoints
)
{
  int targetLaneId = currBehavior.lane.id;
  switch (nextBehaviorState) {
    case BehaviorState::kLaneKeeping:
      return _GenerateLaneKeepingPath(
        currBehavior,
        targetSpeed,
        carState,
        prevPath,
        naviMap,
        endPathFrenetPose,
        numPreservedWaypoints);

    case BehaviorState::kLeftLaneChangePreparation:
      targetLaneId = _GetLaneId(currBehavior.lane.id, "left");
      return _GenerateLaneChangingPreparationPath(
        targetLaneId,
        currBehavior,
        carState,
        prevPath,
        naviMap,
        endPathFrenetPose,
        numPreservedWaypoints);

    case BehaviorState::kLeftLaneChange:
      targetLaneId = _GetLaneId(currBehavior.lane.id, "left");
      return _GenerateLaneChangingPath(
        targetLaneId,
        currBehavior,
        carState,
        prevPath,
        naviMap,
        endPathFrenetPose,
        numPreservedWaypoints);

    case BehaviorState::kRightLaneChangePreparation:
      targetLaneId = _GetLaneId(currBehavior.lane.id, "right");
      return _GenerateLaneChangingPreparationPath(
        targetLaneId,
        currBehavior,
        carState,
        prevPath,
        naviMap,
        endPathFrenetPose,
        numPreservedWaypoints);

    case BehaviorState::kRightLaneChange:
      targetLaneId = _GetLaneId(currBehavior.lane.id, "right");
      return _GenerateLaneChangingPath(
        targetLaneId,
        currBehavior,
        carState,
        prevPath,
        naviMap,
        endPathFrenetPose,
        numPreservedWaypoints);

    default:
      return Path();
  }
}

} // namespace pathplanning
