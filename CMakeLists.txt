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

#指定库路径
file(GLOB_RECURSE Opencv2.7_LIB "/usr/lib/python2.7/config-x86_64-linux-gnu/*.so")
#指定头文件路径
set(Opencv2.7_INLCUDE_DIRS "/usr/include/python2.7")
#添加头文件到工程
include_directories(include
        ${Opencv2.7_INLCUDE_DIRS}
        )
# 通常这样设置就可以，如果你的项目包含qrc文件，那么需要将它们单独添加进来
aux_source_directory(. DIRS_SRCS)


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

target_link_libraries(filter_position_opt PRIVATE osqp::osqp ${Opencv2.7_LIB})


