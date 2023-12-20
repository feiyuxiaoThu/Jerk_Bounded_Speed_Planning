#include "optimizer.h"

Optimizer::Optimizer(const OptimizerSolver& solver_num, const BaseSolver::OptimizerParam& param)
{
    if(solver_num == OSQP_QP)
        solver_ = std::make_shared<osqp::QPSolver>(param);
    else if(solver_num == PIQP_QP)
        solver_ = std::make_shared<piqp::QPSolver>(param);
    
}

bool Optimizer::solveqp(const double& initial_vel,
                           const double& initial_acc,
                           const double& dt,
                           const std::vector<double>& ref_s,
                           const std::vector<double>& max_s,
                           const std::vector<double>& min_s,
                           BaseSolver::OutputInfo& output)
{
    bool is_success = false;

    
    is_success = solver_->solve(initial_vel, initial_acc, dt, ref_s, max_s, min_s, output);
    /*
    if(solver_type == 2){
        is_success = solver_->solvelat(initial_vel, initial_acc, dt, ref_s, max_s, min_s, output);
    }
    else{
        is_success = solver_->solvelon(initial_vel, initial_acc, dt, ref_s, max_s, min_s, output);
    }
    */

    return is_success;
}


