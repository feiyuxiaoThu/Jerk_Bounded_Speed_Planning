#ifndef ASTAR_SPEED_PLANNING_ASTAR_PLANNER_H
#define ASTAR_SPEED_PLANNING_ASTAR_PLANNER_H

#include <iostream>
#include <set>
#include <chrono>
#include <cstring>
#include <fstream>
#include <vector>
#include "astar_node.h"
#include "node_util.h"

class AStarPlanner
{
public:
    AStarPlanner() = default;

    void run(const double weight_v,
             const double t_max,
             const double dt,
             const double goal_s,
             const std::vector<double>& da_list,
             const double obs_t_min,
             const double obs_t_max,
             const double obs_s_min,
             const double obs_s_max,
             const std::string& filename);

    double calcSpeedLimit(const double s);
    double calcRefSpeed(const double s);
    bool isOccupied(const double obs_s_min,
                    const double obs_s_max,
                    const double obs_t_min,
                    const double obs_t_max,
                    double s,
                    double t);
};

#endif //ASTAR_SPEED_PLANNING_ASTAR_PLANNER_H
