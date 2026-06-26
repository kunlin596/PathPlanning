cmake_minimum_required(VERSION 3.16)

project(PathPlanning LANGUAGES CXX)

# ---- Build settings -------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# The project libraries are SHARED, so every (possibly fetched, static)
# dependency must be position-independent.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

add_compile_options(-Wall -Wextra)

include(FetchContent)

# ---- Dependencies ---------------------------------------------------------
# System packages are preferred; anything missing is fetched at configure
# time so the build works on a bare toolchain. uWebSockets and libuv are the
# only deps that must be installed manually (see README) -- they gate the
# server executable, not the libraries or unit tests.

# spdlog supplies logging and, transitively, fmt. The project uses fmt only
# through spdlog's bundled copy (see src/log.h), so:
#   * a system spdlog is normally built against an external fmt -> require fmt;
#   * a fetched spdlog ships its own bundled fmt -> no separate fmt needed.
set(FMT_TARGET "")
find_package(spdlog 1.9 QUIET)
if(spdlog_FOUND)
  find_package(fmt REQUIRED)
  set(FMT_TARGET fmt::fmt)
else()
  # v1.10.0 bundles fmt 8.1.1, matching the implicit operator<< formatting the
  # code relies on (and the fmt the system package provides). Newer spdlog
  # bundles fmt >= 9, which dropped that implicit path.
  message(STATUS "spdlog not found on system -- fetching v1.10.0 (with bundled fmt 8)")
  set(SPDLOG_FMT_EXTERNAL OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG v1.10.0)
  FetchContent_MakeAvailable(spdlog)
endif()

find_package(Eigen3 3.3 QUIET NO_MODULE)
if(NOT Eigen3_FOUND)
  message(STATUS "Eigen3 not found on system -- fetching")
  set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
  set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "" FORCE)
  set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(Eigen3
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.4.0)
  FetchContent_MakeAvailable(Eigen3)
endif()

find_package(Boost REQUIRED COMPONENTS program_options filesystem)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 REQUIRED)

# Server-only dependencies. Resolved here but not REQUIRED, so the libraries
# and unit tests still configure when they are absent (see the server block).
find_package(OpenSSL)
find_package(ZLIB)

option(USE_OPENMP "Enable OpenMP acceleration for path planning" ON)
if(USE_OPENMP)
  find_package(OpenMP)
  if(OpenMP_CXX_FOUND)
    message(STATUS "OpenMP enabled for path planning")
  else()
    message(STATUS "OpenMP requested but not found -- continuing without it")
  endif()
endif()

# libuv / uWebSockets: system libraries, optional at configure time.
find_library(UV_LIB NAMES uv libuv)
find_library(UWS_LIB NAMES uWS libuWS)

# ---- Libraries ------------------------------------------------------------
add_library(common SHARED
  src/configuration.cpp
  src/vehicle.cpp
  src/map.cpp)
target_include_directories(common PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(common PUBLIC spdlog::spdlog ${FMT_TARGET} Eigen3::Eigen)

add_library(perception SHARED src/perception.cpp)
target_link_libraries(perception PUBLIC common)

add_library(tracker SHARED src/tracker.cpp)
target_link_libraries(tracker PUBLIC perception)

add_library(trajectory SHARED
  src/path.cpp
  src/jmt.cpp
  src/collision_checker.cpp
  src/ptg.cpp)
target_link_libraries(trajectory
  PUBLIC common tracker pybind11::embed
  PRIVATE Boost::filesystem)
if(USE_OPENMP AND OpenMP_CXX_FOUND)
  target_link_libraries(trajectory PUBLIC OpenMP::OpenMP_CXX)
endif()

# ---- Server executable ----------------------------------------------------
# Requires uWebSockets, libuv, OpenSSL and zlib. Skipped (with a clear notice)
# when any are absent so the libraries and unit tests still build.
if(UV_LIB AND UWS_LIB AND OPENSSL_FOUND AND ZLIB_FOUND)
  add_executable(pathplanningserver src/system.cpp src/main.cpp)
  target_link_libraries(pathplanningserver
    PRIVATE
    trajectory
    Boost::program_options
    Boost::filesystem
    OpenSSL::SSL
    ZLIB::ZLIB
    ${UV_LIB}
    ${UWS_LIB})
else()
  message(WARNING
    "Skipping the 'pathplanningserver' target -- missing one of: "
    "uWebSockets (libuWS), libuv, OpenSSL, zlib. Install them to build the "
    "server (see README).")
endif()

# ---- Tests ----------------------------------------------------------------
enable_testing()

find_package(GTest QUIET)
if(NOT GTest_FOUND)
  message(STATUS "GTest not found on system -- fetching")
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_Declare(googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0)
  FetchContent_MakeAvailable(googletest)
endif()

add_executable(test_math src/tests/test_math.cpp)
target_link_libraries(test_math PRIVATE common GTest::gtest GTest::gtest_main)

add_executable(test_jmt src/tests/test_jmt.cpp)
target_link_libraries(test_jmt PRIVATE trajectory GTest::gtest GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(test_math)
gtest_discover_tests(test_jmt)
