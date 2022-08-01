#include <iostream>
#include <vector>
#include <iomanip>
#include <chrono>
#include "interpolate.h"
#include "optimizer.h"
#include "utils.h"
#include "obstacle.h"
#include "scenario_generator.h"
#include <iostream>
//#include "solver/base_solver.h"

#include<fstream>

int main()
{
    const std::string current_dir = std::string(RESULT_DIR);
    std::cout << current_dir << std::endl;

    ScenarioGenerator::ScenarioNumber num = ScenarioGenerator:: Acc;

    ScenarioGenerator generator;

    ScenarioGenerator::ScenarioData data;
    generator.generate(num, data);



    /***************************************************/
    /***************** QP Parameter ********************/
    /***************************************************/
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
    

    /***************************************************/
    /********** QP Optimization(Pseudo-Jerk) ***********/
    /***************************************************/
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
