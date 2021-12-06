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
find_package(Boost COMPONENTS program_options filesystem REQUIRED)

OPTION(USE_OPENMP "Use OpenMP" ON)
if (USE_OPENMP)
  message(STATUS "USE_OPENMP=${USE_OPENMP}, OpenMP acceleration will be enabled for path planning.")
  find_package(OpenMP)
  if (OpenMP_CXX_FOUND)
      message(STATUS "Found OpenMP")
      set(OpenMP_LIBS OpenMP::OpenMP_CXX)
      message(STATUS "OpenMP Libs ${OpenMP_LIBS}")
  endif()
endif()

find_package(Python3 COMPONENTS Development REQUIRED)
include_directories(${Python3_INCLUDE_DIRS})
find_package(pybind11 REQUIRED)

add_library(
  common
  SHARED
  src/configuration.cpp
  src/vehicle.cpp
  src/map.cpp
)

target_link_libraries(
  common
  PUBLIC
  spdlog::spdlog
)

add_library(
  perception
  SHARED
  src/perception.cpp
)

target_link_libraries(
  perception
  PUBLIC
  common
)

add_library(
  tracker
  SHARED
  src/tracker.cpp
)

target_link_libraries(
  tracker
  PUBLIC
  perception
)

add_library(
  trajectory
  SHARED
  src/path.cpp
  src/jmt.cpp
  src/collision_checker.cpp
  src/ptg.cpp
)

target_link_libraries(
  trajectory
  PUBLIC
  common
  tracker
  ${OpenMP_LIBS}
  ${Python3_LIBRARIES}
  PRIVATE
  ${Boost_FILESYSTEM_LIBRARY}
)

add_executable(
  pathplanningserver
  src/system.cpp
  src/main.cpp)

target_link_libraries(
  pathplanningserver
  PRIVATE
  pybind11::embed
  Boost::program_options
  z
  ssl
  uv
  uWS
  trajectory)

enable_testing()
find_package(GTest REQUIRED)

add_executable(test_math src/tests/test_math.cpp)
target_link_libraries(
  test_math
  PRIVATE
  common
  gtest
  gtest_main
)

add_executable(test_jmt src/tests/test_jmt.cpp)
target_link_libraries(
  test_jmt
  PRIVATE
  trajectory
  gtest
  gtest_main
)

include(GoogleTest)
gtest_discover_tests(test_jmt)
gtest_discover_tests(test_math)
