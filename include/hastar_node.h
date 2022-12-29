#ifndef ASTAR_SPEED_PLANNING_HASTAR_NODE_H
#define ASTAR_SPEED_PLANNING_HASTAR_NODE_H

#include "base_node.h"

class HAStarNode : public BaseNode
{
public:
    HAStarNode(const double s,
               const double t,
               const double v,
               const double goal_s,
               const int s_d,
               const int t_d,
               const int v_d,
               HAStarNode* parent = nullptr);

    int s_d_;
    int t_d_;
    int v_d_;
    HAStarNode* parent_node_;
};

#endif //ASTAR_SPEED_PLANNING_HASTAR_NODE_H
