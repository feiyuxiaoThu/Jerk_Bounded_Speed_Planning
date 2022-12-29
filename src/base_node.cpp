#include "base_node.h"

BaseNode::BaseNode(const double s,
                   const double t,
                   const double v,
                   const double goal_s)
                   : s_(s), t_(t), v_(v), actual_cost_(0.0)
{
    heuristic_cost_ = calcHeuristicCost(goal_s);
}

double BaseNode::calcHeuristicCost(const double goal_s)
{
    return 10*std::pow(s_-goal_s, 2);
}

bool BaseNode::isGoal(const double goal_s)
{
    return std::fabs(goal_s - s_) < 1e-6 || goal_s < s_;
}

