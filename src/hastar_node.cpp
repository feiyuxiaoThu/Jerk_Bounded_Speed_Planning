#include "hastar_node.h"

HAStarNode::HAStarNode(const double s,
                       const double t,
                       const double v,
                       const double goal_s,
                       const int s_d,
                       const int t_d,
                       const int v_d,
                       HAStarNode *parent)
                       : BaseNode(s, t, v, goal_s), s_d_(s_d), t_d_(t_d), v_d_(v_d), parent_node_(parent)
{
}
