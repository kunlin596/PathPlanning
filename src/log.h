#ifndef PATHPLANNING_LOG_H
#define PATHPLANNING_LOG_H

#include <spdlog/fmt/ostr.h>  // must be included
#include <spdlog/spdlog.h>

#include <array>
#include <vector>

inline std::ostream &operator<<(std::ostream &out,
                                const std::array<double, 2> &arr) {
  return out << fmt::format("[{:7.3f}, {:7.3f}]", arr[0], arr[1]);
}

inline std::ostream &operator<<(std::ostream &out,
                                const std::array<double, 3> &arr) {
  return out << fmt::format("[{:7.3f}, {:7.3f}, {:7.3f}]", arr[0], arr[1],
                            arr[2]);
}

#endif
