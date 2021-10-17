#ifndef PATHPLANNING_MATH_H
#define PATHPLANNING_MATH_H

#include <cmath>

namespace pathplanning {

inline double deg2rad(double x) { return x * M_PI / 180.0; }
inline double rad2deg(double x) { return x * 180.0 / M_PI; }

inline double GetDistance(double x1, double y1, double x2, double y2) {
  return std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

} // end of pathplanning

#endif
