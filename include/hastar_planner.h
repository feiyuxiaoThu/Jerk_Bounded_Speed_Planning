#ifndef ASTAR_SPEED_PLANNING_HASTAR_PLANNER_H
#define ASTAR_SPEED_PLANNING_HASTAR_PLANNER_H

#include <iostream>
#include <set>
#include <chrono>
#include <cstring>
#include <fstream>
#include "hastar_node.h"
#include "node_util.h"

class HAStarPlanner
{
public:
    HAStarPlanner() = default;

    void run(const double weight_v,
             const double t_max,
             const double ds,
             const double dt,
             const double dv,
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

#endif //ASTAR_SPEED_PLANNING_HASTAR_PLANNER_H
