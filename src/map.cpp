#include "map.h"

#include <boost/assert.hpp>
#include <fstream>
#include <iostream>
#include <sstream>

#include "math.h"
#include "spline.h"

namespace pathplanning {

struct Map::Impl
{
  tk::spline sXSpline;
  tk::spline sYSpline;
  tk::spline sDxSpline;
  tk::spline sDySpline;
};

void
Map::Read(const std::string& filename)
{
  std::ifstream in(filename.c_str(), std::ifstream::in);

  // Deserialize map data from file
  std::string line;
  while (getline(in, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float dx;
    float dy;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> dx;
    iss >> dy;
    _x.push_back(x);
    _y.push_back(y);
    _s.push_back(s);
    _dx.push_back(dx);
    _dy.push_back(dy);
  }
  BOOST_ASSERT(_x.size() > 0);
  BOOST_ASSERT(_y.size() > 0);

  _s.push_back(_s[1] + _s[_s.size() - 1]);
  _x.push_back(_x[0]);
  _y.push_back(_y[0]);
  _dx.push_back(_dx[0]);
  _dy.push_back(_dy[0]);

  // Smoothing the map waypoints
  _pImpl->sXSpline.set_points(_s, _x);
  _pImpl->sYSpline.set_points(_s, _y);
  _pImpl->sDxSpline.set_points(_s, _dx);
  _pImpl->sDySpline.set_points(_s, _dy);
}

Map::Map()
  : _pImpl(std::make_shared<Impl>())
{}

Map::Map(const std::string& filename)
  : _pImpl(std::make_shared<Impl>())
{
  Read(filename);
}

int
Map::GetClosestWaypoint(double x, double y) const
{
  double minDist = std::numeric_limits<double>::max();
  int closestWaypointIndex = 0;

  for (int i = 0; i < _x.size(); ++i) {
    double dist = GetDistance(x, y, _x[i], _y[i]);
    if (dist < minDist) {
      minDist = dist;
      closestWaypointIndex = i;
    }
  }

  return closestWaypointIndex;
}

// Returns next waypoint of the closest waypoint
int
Map::GetNextWaypoint(double x, double y, double theta) const
{
  int closestWaypointIndex = GetClosestWaypoint(x, y);
  double heading = std::atan2((_y[closestWaypointIndex] - y), (_x[closestWaypointIndex] - x));
  double angle = std::fabs(theta - heading);

  angle = std::min(2.0 * M_PI - angle, angle);

  if (angle > M_PI / 2.0) {
    ++closestWaypointIndex;
    if (closestWaypointIndex == _x.size()) {
      closestWaypointIndex = 0;
    }
  }

  return closestWaypointIndex;
}

std::array<double, 2>
Map::GetSD(double x, double y, double theta) const
{
  int nextWaypointIndex = GetNextWaypoint(x, y, theta);
  int prevWaypointIndex = nextWaypointIndex - 1;

  if (nextWaypointIndex == 0) {
    prevWaypointIndex = _x.size() - 1;
  }

  double n_x = _x[nextWaypointIndex] - _x[prevWaypointIndex];
  double n_y = _y[nextWaypointIndex] - _y[prevWaypointIndex];
  double x_x = x - _x[prevWaypointIndex];
  double x_y = y - _y[prevWaypointIndex];

  // find the projection of x onto n
  double projNorm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double projX = projNorm * n_x;
  double projY = projNorm * n_y;

  double frenetD = GetDistance(x_x, x_y, projX, projY);

  // see if d value is positive or negative by comparing it to a center point
  double centerX = 1000.0 - _x[prevWaypointIndex];
  double centerY = 2000.0 - _y[prevWaypointIndex];
  double centerToPos = GetDistance(centerX, centerY, x_x, x_y);
  double centerToRef = GetDistance(centerX, centerY, projX, projY);

  if (centerToPos <= centerToRef) {
    frenetD *= -1.0;
  }

  // calculate s value
  double frenetS = 0.0;
  for (int i = 0; i < prevWaypointIndex; ++i) {
    frenetS += GetDistance(_x[i], _y[i], _x[i + 1], _y[i + 1]);
  }

  frenetS += GetDistance(0, 0, projX, projY);

  return { frenetS, frenetD };
}

std::array<double, 2>
Map::GetXY(double s, double d) const
{
  // Default implementation comes with upstream, which provides low resolution.
  //
  // int prevWaypointIndex = -1;

  // while (s > (_s[prevWaypointIndex + 1] - 1e-6) &&
  //        (prevWaypointIndex < static_cast<int>(_s.size() - 1))) {
  //   ++prevWaypointIndex;
  // }

  // int waypoint2 = (prevWaypointIndex + 1) % _x.size();

  // double heading = std::atan2((_y[waypoint2] - _y[prevWaypointIndex]),
  //                             (_x[waypoint2] - _x[prevWaypointIndex]));
  // // the x,y,s along the segment
  // double segS = (s - _s[prevWaypointIndex]);

  // double segX = _x[prevWaypointIndex] + segS * cos(heading);
  // double segY = _y[prevWaypointIndex] + segS * sin(heading);

  // double perpHeading = heading - M_PI / 2.0;

  // double x = segX + d * std::cos(perpHeading);
  // double y = segY + d * std::sin(perpHeading);

  // return { x, y };
  return { GetX(s, d), GetY(s, d) };
}

double
Map::GetX(double s, double d) const
{
  s = std::fmod(s, _s[_s.size() - 1]);
  return _pImpl->sXSpline(s) + d * _pImpl->sDxSpline(s);
}

double
Map::GetY(double s, double d) const
{
  s = std::fmod(s, _s[_s.size() - 1]);
  return _pImpl->sYSpline(s) + d * _pImpl->sDySpline(s);
}

double
GetKMFromMile(double mile)
{
  return mile / 2.24;
}

double
GetMileFromKM(double km)
{
  return km * 2.24;
}
std::array<double, 2>
ComputeFrenetVelocity(const Map& map,
                      const std::array<double, 2>& pos,
                      const std::array<double, 2>& vel,
                      const std::array<double, 2>& sd,
                      const double dt)
{
  using Eigen::Vector2d;
  using std::array;

  Vector2d velDir = { vel[0], vel[1] };
  double speed = velDir.norm();
  if (speed < 1e-6) {
    return { 0.0, 0.0 };
  }

  velDir /= speed;

  // Hard code the local tangential (Frenet) frame by 0.1 meter.
  Vector2d frenetFrameHeadingVec = { sd[0] + 0.1, sd[1] };
  auto tempFrenetFrameHeadingVec = map.GetXY(frenetFrameHeadingVec[0], frenetFrameHeadingVec[1]);

  Vector2d localFrameOrigin = { pos[0], pos[1] };
  Vector2d localFrameDir = { tempFrenetFrameHeadingVec[0], tempFrenetFrameHeadingVec[1] };
  localFrameDir = localFrameDir - localFrameOrigin;
  localFrameDir /= localFrameDir.norm();

  double headingAngle = std::acos(localFrameDir.transpose() * velDir);
  array<double, 2> sdVel = { speed * std::cos(headingAngle), speed * std::sin(headingAngle) };
  return sdVel;
}

} // namespace pathplanning
