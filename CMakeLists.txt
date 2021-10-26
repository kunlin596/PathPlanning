project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++17)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    include_directories(/usr/local/include)
    link_directories(/usr/local/lib)
    if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm64")
      include_directories(/opt/homebrew/Cellar/openssl@3/3.0.0_1/include)
      include_directories(/opt/homebrew/include)
      link_directories(/opt/homebrew/Cellar/openssl@3/3.0.0_1/lib)
      link_directories(/opt/homebrew/lib)
    else()
      include_directories(/usr/local/opt/openssl/include)
      link_directories(/usr/local/opt/openssl/lib)
      link_directories(/usr/local/Cellar/libuv/1.11.0/lib)
    endif()
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

find_package(spdlog REQUIRED)

add_library(
  driving
  SHARED
  src/configuration.cpp
  src/path.cpp
  src/vehicle.cpp
  src/jmt.cpp
  src/traj_evaluator.cpp
  src/ptg.cpp
  src/map.cpp
  src/tracker.cpp
  src/behaviorplanner.cpp
)

target_link_libraries(
  driving
  PUBLIC
  spdlog::spdlog
)

add_executable(
  pathplanningserver
  src/system.cpp
  src/main.cpp)

target_link_libraries(
  pathplanningserver
  PRIVATE
  z
  ssl
  uv
  uWS
  driving)

add_executable(test_math src/test_math.cpp)
target_link_libraries(
  test_math
  PRIVATE
  spdlog::spdlog
)

add_executable(test_ptg src/test_ptg.cpp)
target_link_libraries(
  test_ptg
  PRIVATE
  driving
)
