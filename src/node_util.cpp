//
// Created by 清水豊禾 on 2020/05/02.
//

#include "node_util.h"

AStarNode* NodeUtil::findNodeOnList(const std::set<AStarNode*>& node_list,
                                    const double s,
                                    const double t,
                                    const double v)
{
    for(AStarNode* node : node_list)
        if(std::fabs(node->s_-s) < 1e-6 && std::fabs(node->t_-t) < 1e-6 && std::fabs(node->v_-v) < 1e-6)
            return node;

    return nullptr;
}

HAStarNode* NodeUtil::findNodeOnList(const std::set<HAStarNode*>& node_list,
                                     const int s_d,
                                     const int t_d,
                                     const int v_d)
{
    for(HAStarNode* node : node_list)
        if(node->s_d_ == s_d && node->t_d_ == t_d && node->v_d_ == v_d)
            return node;

    return nullptr;
}
