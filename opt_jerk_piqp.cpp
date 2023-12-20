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

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

#include<fstream>

int main()
{
    const std::string current_dir = std::string(RESULT_DIR);
    std::cout << current_dir << std::endl;

    ScenarioGenerator::ScenarioNumber num = ScenarioGenerator:: Cutout;

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

    param.ref_v = 18.0; 

    param.over_s_weight = 1;
    param.over_v_weight = 5;
    param.over_a_weight = 50;
    param.over_j_weight = 10;
    param.over_sref_weight = 0.1;
    param.over_vref_weight = 0.1;
    param.over_send_weight = 0.1;
    param.over_vend_weight = 0.1;
    param.over_aend_weight = 0.1;

    

    /***************************************************/
    /********** QP Optimization(Pseudo-Jerk) ***********/
    /***************************************************/
    Optimizer qp_optimizer(Optimizer::OptimizerSolver::PIQP_QP, param);
    BaseSolver::OutputInfo qp_output;
    qp_output.position = data.ref_position_;

    std::chrono::system_clock::time_point qp_start, qp_end;
    qp_start = std::chrono::system_clock::now();

    bool qp_result = qp_optimizer.solveqp(data.v0_, data.a0_, data.dt_, data.ref_position_, data.max_position_,
    data.min_position_,qp_output);

    qp_end = std::chrono::system_clock::now();
    double qp_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(qp_end-qp_start).count();

    //test lp
    

    
    if(!qp_result)
    {
        std::cerr << "QP Solver has Error" << std::endl;
        return -1;
    }
    else{
        // csv writer
        std::cerr << "QP Solver use time = " << qp_elapsed  << " ms" <<std::endl;
        std::string obs_filenamenew = current_dir + "/result/Opt_vel_piqp.csv";
        Utils::outputToFile(obs_filenamenew, data.ref_position_,data.max_position_,data.min_position_,qp_output);
        
        //PLot
        plt::figure();
        plt::plot(qp_output.time,qp_output.position,{{"label","op position"}});
        plt::plot(qp_output.time,data.max_position_,{{"label","upper position"}});
        plt::plot(qp_output.time,data.min_position_,{{"label","lower position"}});

        plt::title("Position");
        plt::legend();
        //plt::show();
        plt::save("../result/position_piqp.pdf");

        plt::figure();
        plt::plot(qp_output.time,qp_output.velocity,{{"label","op velocity"}});

        plt::title("velocity");
        plt::legend();
        plt::save("../result/velocity_piqp.pdf");

        plt::figure();
        plt::plot(qp_output.time,qp_output.acceleration,{{"label","op acceleration"}});
        plt::plot(qp_output.time,qp_output.jerk,{{"label","op jerk"}});

        plt::title("acceleration");
        plt::legend();
        plt::save("../result/acceleration_piqp.pdf");
    }
    
   


    
    return 0;
}
