#ifndef PATHPLANNING_LOG_H
#define PATHPLANNING_LOG_H

// For debug log to appear, See https://github.com/gabime/spdlog/issues/1318
#ifndef SPDLOG_ACTIVE_LEVEL
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE
#endif

#include <spdlog/fmt/bundled/ranges.h>
#include <spdlog/fmt/ostr.h> // must be included
#include <spdlog/spdlog.h>

#include <array>
#include <iostream>
#include <vector>

#include "Eigen-3.3/Eigen/Dense"

// NOTE: Workaround for std array.

namespace pathplanning {
template<typename _DType, uint32_t _Size>
inline std::ostream&
_Print(std::ostream& out, const std::array<_DType, _Size>& arr)
{
  out << std::string("[");
  for (uint32_t i = 0; i < _Size; ++i) {
    out << fmt::format("{:7.3f}", arr[i]);
    if (i != _Size - 1) {
      out << std::string(", ");
    }
  }
  return out << std::string("]");
}

inline std::ostream&
operator<<(std::ostream& out, const std::array<double, 2>& arr)
{
  return _Print<double, 2>(out, arr);
}

inline std::ostream&
operator<<(std::ostream& out, const std::array<double, 3>& arr)
{
  return _Print<double, 3>(out, arr);
}

inline std::ostream&
operator<<(std::ostream& out, const std::array<double, 4>& arr)
{
  return _Print<double, 4>(out, arr);
}

inline std::ostream&
operator<<(std::ostream& out, const std::array<double, 5>& arr)
{
  return _Print<double, 5>(out, arr);
}

inline std::ostream&
operator<<(std::ostream& out, const std::array<double, 6>& arr)
{
  return _Print<double, 6>(out, arr);
}

static const Eigen::IOFormat HeavyFmt(Eigen::StreamPrecision, 0, ", ", ",\n", "[", "]", "[", "]");

} // namespace pathplanning

#endif
