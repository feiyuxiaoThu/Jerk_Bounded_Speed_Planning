cmake_minimum_required(VERSION 3.15)
project(astar_speed_planning)

set(CMAKE_CXX_STANDARD 14)

include_directories(include)

add_library(astar_lib src/base_node.cpp src/node_util.cpp src/astar_node.cpp src/hastar_node.cpp src/astar_planner.cpp src/hastar_planner.cpp)

add_executable(main src/main.cpp)
target_link_libraries(main astar_lib)


