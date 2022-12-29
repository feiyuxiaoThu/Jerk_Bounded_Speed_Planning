cmake_minimum_required(VERSION 2.8.3)
project(ST_graph)


add_compile_options(-std=c++11)

SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

# find_package( REQUIRED COMPONENTS
#     # roscpp
#     # geometry_msgs
#     pcl_conversions
#     pcl_ros
#     std_msgs
#     # smartcar_msgs
#     sensor_msgs
#     tf
# )

# catkin_package(
#     INCLUDE_DIRS include
#     LIBRARIES 
#     CATKIN_DEPENDS
#         roscpp
#         std_msgs
#         # smartcar_msgs
#         geometry_msgs
# )

include_directories(
    INCLUDE_DIRS include
)

add_executable( DP_Speed_Planner
    src/DP_speed_planner.cpp
)
