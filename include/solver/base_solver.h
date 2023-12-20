#ifndef FILTER_POSITION_OPTIMIZATION_BASE_SOLVER_H
#define FILTER_POSITION_OPTIMIZATION_BASE_SOLVER_H

#include <Eigen/Eigen>
#include <vector>
#include <cmath>
#include <cassert>
#include <iostream>
#include <chrono>
#include "interpolate.h"
#include "solver/osqp_interface/osqp_interface.h"
#include <limits>
#include "solver/piqp/piqp.hpp"

class BaseSolver
{
public:
    struct OptimizerParam
    {
        double max_accel;
        double min_decel;
        double max_jerk;
        double min_jerk;
        double max_vel;
        double min_vel;
        double ref_v; // target final velocity
        //double smooth_weight;
        double over_s_weight;
        double over_v_weight;
        double over_a_weight;
        double over_j_weight;

        double over_sref_weight;
        double over_vref_weight;
        double over_send_weight;
        double over_vend_weight;
        double over_aend_weight;


    
    };

    struct OutputInfo
    {
        std::vector<double> time;
        std::vector<double> position;
        std::vector<double> velocity;
        std::vector<double> acceleration;
        std::vector<double> jerk;

        void reserve(const unsigned int& N)
        {
            time.reserve(N);
            position.reserve(N);
            velocity.reserve(N);
            acceleration.reserve(N);
            jerk.reserve(N);
        }

        void resize(const unsigned int& N)
        {
            time.resize(N);
            position.resize(N);
            velocity.resize(N);
            acceleration.resize(N);
            jerk.resize(N);
        }
    };

    BaseSolver(const OptimizerParam& param) : param_(param) {};

    void setParam(const OptimizerParam& param)
    {
        param_ = param;
    }

    virtual bool solve(const double& initial_vel,
                           const double& initial_acc,
                           const double& dt,
                           const std::vector<double>& ref_s,
                           const std::vector<double>& max_s,
                           const std::vector<double>& min_s,
                           OutputInfo& output) = 0;
    /*
    virtual bool solvelat(const double& initial_vel,
                           const double& initial_acc,
                           const double& dt,
                           const std::vector<double>& ref_s,
                           const std::vector<double>& max_s,
                           const std::vector<double>& min_s,
                           OutputInfo& output) = 0;
   
     
    virtual bool solvelon(const double& initial_vel,
                           const double& initial_acc,
                           const double& dt,
                           const std::vector<double>& ref_s,
                           const std::vector<double>& max_s,
                           const std::vector<double>& min_s,
                           OutputInfo& output) = 0;
    */

protected:
    OptimizerParam param_;
};

#endif //FILTER_POSITION_OPTIMIZATION_BASE_SOLVER_H
