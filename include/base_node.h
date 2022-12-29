#ifndef ASTAR_SPEED_PLANNING_BASE_NODE_H
#define ASTAR_SPEED_PLANNING_BASE_NODE_H

#include <iostream>
#include <cmath>
#include <vector>

class BaseNode
{
public:
    BaseNode(const double s,
             const double t,
             const double v,
             const double goal_s);

    double getScore() { return actual_cost_+heuristic_cost_; }
    double calcHeuristicCost(const double goal_s);
    bool isGoal(const double goal_s);

    double s_;
    double t_;
    double v_;
    double actual_cost_;
    double heuristic_cost_;
};

#endif //ASTAR_SPEED_PLANNING_BASE_NODE_H
