#ifndef ASTAR_SPEED_PLANNING_ASTAR_NODE_H
#define ASTAR_SPEED_PLANNING_ASTAR_NODE_H

#include <iostream>
#include <vector>
#include <cmath>
#include <memory>
#include "base_node.h"

class AStarNode : public BaseNode
{
public:
    AStarNode(const double s,
         const double t,
         const double v,
         const double goal_s,
         AStarNode* parent_node = nullptr);

    AStarNode* parent_node_;
};


#endif //ASTAR_SPEED_PLANNING_ASTAR_NODE_H
