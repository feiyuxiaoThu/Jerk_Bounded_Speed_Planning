#include "hastar_planner.h"

double HAStarPlanner::calcSpeedLimit(const double s)
{
    return 10.0;
}

double HAStarPlanner::calcRefSpeed(const double s)
{
    return 3.0;
}

bool HAStarPlanner::isOccupied(const double obs_s_min,
                              const double obs_s_max,
                              const double obs_t_min,
                              const double obs_t_max,
                              double s,
                              double t)
{
    return obs_s_min <= s && s <= obs_s_max && obs_t_min < t && t < obs_t_max;
}

void HAStarPlanner::run(const double weight_v,
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
                        const std::string& filename)
{
    //Output File Information
    std::ofstream writing_file;
    writing_file.open(filename, std::ios::out);
    writing_file << "s" << "," << "t" << std::endl;

    //node list
    std::set<HAStarNode*> open_node_list;
    std::set<HAStarNode*> closed_node_list;

    //set start node
    double s0 = 0.0;
    double t0 = 0.0;
    double v0 = 0.0;
    int s_d = int(s0/ds);
    int t_d = int(t0/dt);
    int v_d = int(v0/dv);
    HAStarNode start_node(s0, t0, v0, goal_s, s_d, t_d, v_d);
    open_node_list.insert(&start_node);

    std::chrono::system_clock::time_point  start, end;
    start = std::chrono::system_clock::now(); //start
    while(true)
    {
        if(open_node_list.empty())
        {
            std::cout << "We cannot find a solution" << std::endl;
            break;
        }

        //choose least cost node
        HAStarNode* current_node = *open_node_list.begin();
        for(HAStarNode* node : open_node_list)
            if(node->getScore() <= current_node->getScore())
                current_node = node;

        //check current node is in the goal_s
        if(current_node->isGoal(goal_s))
        {
            end = std::chrono::system_clock::now();  // end time
            double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
            std::cout << "Hybrid A Star Calculation Time: " << elapsed << std::endl;

            std::cout << "We reached the goal" << std::endl;
            while(current_node != nullptr)
            {
                //std::cout << "s: " << current_node->s_ << std::endl;
                //std::cout << "t: " << current_node->t_ << std::endl;
                //std::cout << "v: " << current_node->v_ << std::endl;
                writing_file << current_node->s_ << "," << current_node->t_ << std::endl;

                current_node = current_node->parent_node_;
            }
            break;
        }

        //insert current node into closed node list and erase current node from open node list
        closed_node_list.insert(current_node);
        open_node_list.erase(open_node_list.find(current_node));

        //update neighbors
        for(double a_command : da_list)
        {
            double new_s = current_node->s_ + current_node->v_*dt + 0.5*a_command*dt*dt;
            double new_t = current_node->t_ + dt;
            double new_v = current_node->v_ + a_command*dt;
            int new_s_d = static_cast<int>(std::round(new_s/ds));
            int new_t_d = static_cast<int>(std::round(new_t/dt));
            int new_v_d = static_cast<int>(std::round(new_v/dv));

            bool isCollide = isOccupied(obs_s_min, obs_s_max, obs_t_min, obs_t_max, new_s, new_t);
            if(-1e-6 < new_v && new_v<= calcSpeedLimit(new_s) && new_t <= t_max && !isCollide)
            {
                HAStarNode* closed_node = NodeUtil::findNodeOnList(closed_node_list, new_s_d, new_t_d, new_v_d);
                if(closed_node != nullptr)
                    continue;

                double reference_v = calcRefSpeed(new_s);
                double additional_cost = weight_v * std::pow((new_v - reference_v), 2);
                double new_cost = current_node->actual_cost_ + additional_cost;

                HAStarNode* successor = NodeUtil::findNodeOnList(open_node_list, new_s_d, new_t_d, new_v_d);
                if(successor==nullptr)
                {
                    //We don't have this node in the open node list
                    successor = new HAStarNode(new_s, new_t, new_v, goal_s, new_s_d, new_t_d, new_v_d, current_node);
                    successor->actual_cost_ = new_cost;
                    open_node_list.insert(successor);
                }
                else if(new_cost < successor->actual_cost_)
                {
                    /*
                       We already has this node in the open node list, and the node cost is lower than original,
                       so we will update the cost
                     */
                    successor->s_ = new_s;
                    successor->t_ = new_t;
                    successor->v_ = new_v;
                    successor->actual_cost_ = new_cost;
                    successor->heuristic_cost_ = successor->calcHeuristicCost(goal_s);
                    successor->parent_node_ = current_node;
                }
            }
        }
    }
}