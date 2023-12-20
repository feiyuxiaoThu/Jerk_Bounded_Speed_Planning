#ifndef FILTER_POSITION_OPTIMIZATION_OPTIMIZER_H
#define FILTER_POSITION_OPTIMIZATION_OPTIMIZER_H

#include "solver/base_solver.h"
#include "solver/qp_solver_osqp.h"
#include "solver/qp_solver_piqp.h"
#include <memory>

class Optimizer
{
public:
    enum OptimizerSolver
    {
        OSQP_QP = 0,
        PIQP_QP = 1,
        // can add more solvers ...
    };

    Optimizer(const OptimizerSolver& solver_num, const BaseSolver::OptimizerParam& param);

    bool solveqp(const double& initial_vel,
                           const double& initial_acc,
                           const double& dt,
                           const std::vector<double>& ref_s,
                           const std::vector<double>& max_s,
                           const std::vector<double>& min_s,
                           BaseSolver::OutputInfo& output);
    
    


private:
    std::shared_ptr<BaseSolver> solver_;
};

#endif //FILTER_POSITION_OPTIMIZATION_OPTIMIZER_H
