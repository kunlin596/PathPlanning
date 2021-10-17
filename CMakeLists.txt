project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++17)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

find_package(Boost REQUIRED COMPONENTS log)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 
    include_directories(/usr/local/opt/openssl/include)
    include_directories(/usr/local/include)
    link_directories(/usr/local/lib)
    link_directories(/usr/local/opt/openssl/lib)
    link_directories(/usr/local/Cellar/libuv/1.11.0/lib)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

add_library(
  driving
  SHARED
  src/path.cpp
  src/jmt.cpp
  src/ptg.cpp
  src/perception.cpp
  src/behaviorplanner.cpp
  src/map.cpp
  src/system.cpp
)

# set(sources src/helpers.cpp src/pathplanner.cpp src/behaviorplanner.cpp src/main.cpp)
# add_executable(path_planning ${sources})
# target_link_libraries(path_planning z ssl uv uWS Boost::log)
