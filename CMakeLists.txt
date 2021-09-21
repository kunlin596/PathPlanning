project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

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

set(sources src/helpers.cpp src/planner.cpp src/behaviorplanner.cpp src/main.cpp)
add_executable(path_planning ${sources})
target_link_libraries(path_planning z ssl uv uWS Boost::log)
