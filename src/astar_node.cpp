#include "astar_node.h"

AStarNode::AStarNode(const double s,
           const double t,
           const double v,
           const double goal_s,
           AStarNode* parent_node) : BaseNode(s, t, v, goal_s),
           parent_node_(parent_node)
{
}

