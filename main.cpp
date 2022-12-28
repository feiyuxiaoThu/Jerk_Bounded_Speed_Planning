#include <iostream>
#include <vector>
#include <iomanip>
#include <chrono>
#include "interpolate.h"
#include "optimizer.h"
#include "utils.h"
#include "obstacle.h"
#include "scenario_generator.h"


#include "DP_speed_planner.h"
#include <iostream>
//#include "solver/base_solver.h"

#include<fstream>

int main()
{
    // Speed DP
    
    // 加入障碍物
    std::vector<planningQP::Waypoint> points;
    std::shared_ptr<planningQP::DiscretizedPath> path;
    for (double x = 0.0; x < 20; x += 0.1)
    {
        points.emplace_back(x, x, M_PI / 4.0, std::hypot(x, x));
    }
    path = std::make_shared<planningQP::DiscretizedPath>(points);

    planningQP::Box2d obs_box3({1, 1}, M_PI / 4.0, 1.0, 1.0);
    std::vector<planningQP::TrajectoryPoint> traj_points;
    double t = 0.0;
    for (double x = 5.0; x > 0.0; x -= 0.1)
    {
        planningQP::TrajectoryPoint traj_p({x, -x + 5, 3 * M_PI / 4.0, 0}, 0.0, t);
        traj_points.emplace_back(traj_p);
        t += 0.1;
    }

    planningQP::DiscretizedTrajectory obs_trajectory(traj_points);
    planningQP::Obstacle obs1("3", obs_box3, false, obs_trajectory);

    planningQP::Box2d obs_box4({1, 1}, M_PI / 4.0, 1, 1.0);
    planningQP::Obstacle obs2("4", obs_box4, true);

    planningQP::StGraph st_graph(*path, std::vector<planningQP::Obstacle>{obs1, obs2});


    std::vector<planningQP::StBoundary> st_boundaries;

    std::chrono::system_clock::time_point dp_start, dp_end;
    dp_start = std::chrono::system_clock::now();

    st_graph.GetAllObstacleStBoundary(&st_boundaries);

    //////////////////////////////////////////////////////
    // std::vector<StBoundary> st_boundaries_debug;
    // StBoundary st_boundary_1;
    // StBoundary st_boundary_2;
    // StBoundary st_boundary_3;

    // StPoint lower_point(0.0, 0.0);
    // StPoint upper_point(1.0, 1.0);
    // std::vector<StPoint> lower_points;
    // std::vector<StPoint> upper_points;

    // lower_points.push_back(lower_point);
    // upper_points.push_back(upper_point);

    // st_boundary_1.Init("1", lower_points, upper_points);
    // st_boundary_1.Init("2", lower_points, upper_points);
    // st_boundary_1.Init("3", lower_points, upper_points);

    // st_boundaries_debug.push_back(st_boundary_1);
    // st_boundaries_debug.push_back(st_boundary_2);
    // st_boundaries_debug.push_back(st_boundary_3);
    // DP用于速度规划
    
    bool res = planningQP::DP_speed_Search(st_boundaries);

    dp_end = std::chrono::system_clock::now();
    double dp_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(dp_end-dp_start).count();

    std::cout << "End DP_speed =" << res << "and time cost = " << dp_elapsed << " ms" << std::endl;

    

    // Speed QP
    const std::string current_dir = std::string(RESULT_DIR);
    std::cout << current_dir << std::endl;

    ScenarioGenerator::ScenarioNumber num = ScenarioGenerator:: Acc;

    ScenarioGenerator generator;

    ScenarioGenerator::ScenarioData data;
    generator.generate(num, data);




    // Note Parameters should be carefully adjusted according to the specific scenerios
    BaseSolver::OptimizerParam param{};
    param.max_accel = 3.0;
    param.min_decel = -3.0;
    param.max_jerk = 2.0;
    param.min_jerk = -2.0;
    param.max_vel = 33.3;
    param.min_vel = 0.0;

    param.ref_v = 15.0; 

    param.over_s_weight = 0;
    param.over_a_weight = 10;
    param.over_v_weight = 200;
    

    Optimizer qp_optimizer(Optimizer::OptimizerSolver::OSQP_QP, param);
    BaseSolver::OutputInfo qp_output;
    qp_output.position = data.ref_position_;

    std::chrono::system_clock::time_point qp_start, qp_end;
    qp_start = std::chrono::system_clock::now();

    bool qp_result = qp_optimizer.solveqp(data.v0_, data.a0_, data.dt_, data.ref_position_, data.max_position_,
    data.min_position_,qp_output);

    qp_end = std::chrono::system_clock::now();
    double qp_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(qp_end-qp_start).count();

    
    if(!qp_result)
    {
        std::cerr << "QP Solver has Error" << std::endl;
        return -1;
    }
    else{
        // csv writer
        std::cerr << "QP Solver use time = " << qp_elapsed  << " ms" <<std::endl;
        std::string obs_filenamenew = current_dir + "/result/Opt_vel.csv";
        Utils::outputToFile(obs_filenamenew, data.ref_position_,data.max_position_,data.min_position_,qp_output);
        
    }
    


    
    
    return 0;
}
