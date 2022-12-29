#include <iostream>
#include <set>
#include <chrono>
#include <cstring>
#include <fstream>
#include "astar_planner.h"
#include "hastar_planner.h"

int main(int argc, char** argv)
{
    //Output File Information
    std::string astar_filename = "../result_as.csv";
    std::string hastar_filename = "../result_has.csv";

    //Basic Information
    double weight_v = 1.0;
    double goal_s = 60.0;
    double t_max = 20.0;
    double a_max = 2.0;
    double dt = 0.1;
    double ds = 0.2;
    double da = a_max;
    double dv = da*dt;
    std::vector<double> da_list = {da, 0.0, -da};

    //Obstacle information
    double obs_t_min = 4.0;
    double obs_t_max = 6.0;
    double obs_s_min = 20.0;
    double obs_s_max = 22.0;

    AStarPlanner astar_planner;
    astar_planner.run(weight_v, t_max, dt, goal_s, da_list, obs_t_min, obs_t_max, obs_s_min, obs_s_max, astar_filename);

    HAStarPlanner hastar_planner;
    hastar_planner.run(weight_v, t_max, ds, dt, dv, goal_s, da_list, obs_t_min, obs_t_max, obs_s_min, obs_s_max, hastar_filename);

    return 0;
}
