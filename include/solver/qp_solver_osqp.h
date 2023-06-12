#ifndef FILTER_POSITION_OPTIMIZATION_QP_SOLVER_OSQP_H
#define FILTER_POSITION_OPTIMIZATION_QP_SOLVER_OSQP_H

#include "solver/base_solver.h"

namespace osqp
{
    class QPSolver : public BaseSolver
    {
    public:
        QPSolver(const OptimizerParam& param) : BaseSolver(param) {}

        bool solvelat(const double& initial_vel,
                           const double& initial_acc,
                           const double& dt,
                           const std::vector<double>& ref_s,
                           const std::vector<double>& max_s,
                           const std::vector<double>& min_s,
                           OutputInfo& output);

        bool solvelon(const double& initial_vel,
                           const double& initial_acc,
                           const double& dt,
                           const std::vector<double>& ref_s,
                           const std::vector<double>& max_s,
                           const std::vector<double>& min_s,
                           OutputInfo& output);
    };

       
}

#endif //FILTER_POSITION_OPTIMIZATION_QP_SOLVER_OSQP_H