project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++17)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

find_package(Boost REQUIRED COMPONENTS log)

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

add_library(
  driving
  SHARED
  src/path.cpp
  src/vehicle.cpp
  src/jmt.cpp
  src/ptg.cpp
  src/perception.cpp
  src/behaviorplanner.cpp
  src/map.cpp
  src/system.cpp
)

add_executable(pathplanningserver src/main.cpp)
target_link_libraries(
  pathplanningserver
  z ssl uv uWS Boost::log driving)
