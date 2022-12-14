#include "optimizer.h"

Optimizer::Optimizer(const OptimizerSolver& solver_num, const BaseSolver::OptimizerParam& param)
{
    if(solver_num == OSQP_QP)
        solver_ = std::make_shared<osqp::QPSolver>(param);
    
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
    
    is_success = solver_->solveqp(initial_vel, initial_acc, dt, ref_s, max_s, min_s, output);

    return is_success;
}


