#include "astar_planner.h"

double AStarPlanner::calcSpeedLimit(const double s)
{
    return 10.0;
}

double AStarPlanner::calcRefSpeed(const double s)
{
    return 3.0;
}

bool AStarPlanner::isOccupied(const double obs_s_min,
                              const double obs_s_max,
                              const double obs_t_min,
                              const double obs_t_max,
                              double s,
                              double t)
{
    return obs_s_min <= s && s <= obs_s_max && obs_t_min < t && t < obs_t_max;
}

void AStarPlanner::run(const double weight_v,
                       const double t_max,
                       const double dt,
                       const double goal_s,
                       const std::vector<double>& da_list,
                       const double obs_t_min,
                       const double obs_t_max,
                       const double obs_s_min,
                       const double obs_s_max,
                       const std::string& filename)
{
    //file preparation
    std::ofstream writing_file;
    writing_file.open(filename, std::ios::out);
    writing_file << "s" << "," << "t" << std::endl;

    //node list
    std::set<AStarNode*> open_node_list;
    std::set<AStarNode*> closed_node_list;

    AStarNode start_node(0.0, 0.0, 0.0, goal_s);
    open_node_list.insert(&start_node);

    std::chrono::system_clock::time_point  start, end;
    start = std::chrono::system_clock::now(); //start
    while(true)
    {
        if(open_node_list.empty())
        {
            std::cout << "We cannot find a solution" << std::endl;
            return;
        }

        //choose least cost node
        AStarNode* current_node = *open_node_list.begin();
        for(AStarNode* node : open_node_list)
            if(node->getScore() <= current_node->getScore())
                current_node = node;

        //check current node is in the goal_s
        if(current_node->isGoal(goal_s))
        {
            end = std::chrono::system_clock::now();  // end time
            double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
            std::cout << "A Star Planner Calculation Time: " << elapsed << std::endl;

            std::cout << "We reached the goal" << std::endl;
            while(current_node != nullptr)
            {
                //std::cout << "s: " << current_node->s_ << std::endl;
                //std::cout << "t: " << current_node->t_ << std::endl;
                //std::cout << "v: " << current_node->v_ << std::endl;
                writing_file << current_node->s_ << "," << current_node->t_ << std::endl;

                current_node = current_node->parent_node_;
            }

            return;
        }

        closed_node_list.insert(current_node);
        open_node_list.erase(open_node_list.find(current_node));

        for(double da : da_list)
        {
            double new_v = current_node->v_ + da*dt;
            double new_t = current_node->t_ + dt;
            double new_s = current_node->s_ + current_node->v_*dt + 0.5*da*dt*dt;

            bool isCollide = isOccupied(obs_s_min, obs_s_max, obs_t_min, obs_t_max, new_s, new_t);
            if(-1e-6 < new_v && new_v<= calcSpeedLimit(new_s) && new_t <= t_max && !isCollide)
            {
                AStarNode* closed_node = NodeUtil::findNodeOnList(closed_node_list, new_s, new_t, new_v);
                if(closed_node != nullptr)
                    continue;

                double reference_v = calcRefSpeed(new_s);
                double additional_cost = weight_v * std::pow((new_v - reference_v), 2);
                double new_cost = current_node->actual_cost_ + additional_cost;

                AStarNode* successor = NodeUtil::findNodeOnList(open_node_list, new_s, new_t, new_v);
                if(successor==nullptr)
                {
                    //We don't have this node in the open node list
                    successor = new AStarNode(new_s, new_t, new_v, goal_s, current_node);
                    successor->actual_cost_ = new_cost;
                    open_node_list.insert(successor);
                }
                else if(new_cost < successor->actual_cost_)
                {
                    /*
                       We already has this node in the open node list, and the node cost is lower than original,
                       so we will update the cost
                     */
                    successor->parent_node_ = current_node;
                    successor->actual_cost_ = new_cost;
                }
            }
        }
    }

    return;
}