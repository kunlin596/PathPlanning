#pragma once
#include "json.hpp"
#include <fstream>

namespace pathplanning {
namespace utils {

inline void
WriteJson(const std::string& filename, const nlohmann::json& j)
{
  std::ofstream o(filename);
  o << std::setw(4) << j << std::endl;
}
} // namespace utils
} // namespace pathplanning
