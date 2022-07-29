cmake_minimum_required(VERSION 3.3)
project(filter_position_optimization)
set(CMAKE_CXX_STNDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

option(CXX "enable C++ compilation" ON)
if(CXX)
    enable_language(CXX)
endif()

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

add_definitions(-DRESULT_DIR="${CMAKE_SOURCE_DIR}")

include_directories(include)

find_package(Boost REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


find_package(osqp REQUIRED)
get_target_property(osqp_INCLUDE_DIR osqp::osqpstatic INTERFACE_INCLUDE_DIRECTORIES)
include_directories(${osqp_INCLUDE_DIR})

if(MSVC AND MT)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd")
    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /MT")
    set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} /MTd")
endif()

add_executable(filter_position_opt main.cpp
                                   src/interpolate.cpp
                                   src/solver/qp_solver_osqp.cpp
                                   src/solver/osqp_interface/csc_matrix_conv.cpp
                                   src/solver/osqp_interface/osqp_interface.cpp
                                   src/utils.cpp
                                   src/optimizer.cpp
                                   src/obstacle.cpp
                                   src/scenario_generator.cpp)

target_link_libraries(filter_position_opt PRIVATE osqp::osqp)


