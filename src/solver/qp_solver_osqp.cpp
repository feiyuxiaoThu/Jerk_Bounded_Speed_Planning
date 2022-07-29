#include "solver/qp_solver_osqp.h"

namespace osqp
{
    bool QPSolver::solveqp(const double& initial_vel,
                           const double& initial_acc,
                           const double& dt,
                           const std::vector<double>& ref_s,
                           const std::vector<double>& max_s,
                           const std::vector<double>& min_s,
                           OutputInfo& output)
    {
        /*
         * x = [b[0], b[1], ..., b[N] | a[0], a[1], .... a[N] | delta[0], ..., delta[N]
         *      | sigma[0], sigma[1], ...., sigma[N] | gamma[0], gamma[1], ..., gamma[N] ]
         * b[i]: velocity^2
         * delta: 0 < b[i]-delta[i] < max_vel[i]*max_vel[i]
         * sigma: amin < a[i] - sigma[i] < amax
         * gamma: jerk_min/ref_vel[i] < pseudo_jerk[i] - gamma[i] < jerk_max/ref_vel[i]
         */

        // Create Solver
        osqp::OSQPInterface qp_solver_;
        qp_solver_.updateMaxIter(1000);
        qp_solver_.updateRhoInterval(0);  // 0 means automatic
        qp_solver_.updateEpsRel(1.0e-4);  // def: 1.0e-4
        qp_solver_.updateEpsAbs(1.0e-8);  // def: 1.0e-4
        qp_solver_.updateVerbose(false);

        long N = static_cast<long>(ref_s.size());
        const long var_size = 3*N;
        const long size_cons_part1 = 2*N+1;
        const long size_cons_part2 = 4*N-1;
        const long constraint_size = size_cons_part1 + size_cons_part2;
        const double amax = param_.max_accel;
        const double amin = param_.min_decel;
        const double vmax = param_.max_vel;
        const double vmin = param_.min_vel;
        const double jmax = param_.max_jerk;
        const double jmin = param_.min_jerk;
        const double over_s_weight = param_.over_s_weight;
        const double over_v_weight = param_.over_v_weight;
        const double over_a_weight = param_.over_a_weight;

        const double ref_v = param_.ref_v;
        const double delta_t = dt;

        // Hessian
        Eigen::MatrixXd hessian = Eigen::MatrixXd::Zero(var_size, var_size);
        for(int i=0; i<N; ++i)
        {
            // no 2 and gradient with 2 !!!! check
            hessian(i,i) = over_s_weight;
            hessian(i+N, i+N) = over_v_weight;
            hessian(i+2*N, i+2*N) = over_v_weight;
        }

        // Gradient
        std::vector<double> gradient(var_size, 0.0);
        for(long i=0; i<N; ++i)
        {
            gradient[i] = -2*ref_s[i]*over_s_weight;
            gradient[i+N] = -2*ref_v*over_v_weight;
            // note ref_a == 0.0;
        }

        // Constraint Matrix
        Eigen::MatrixXd constraint_matrix = Eigen::MatrixXd::Zero(constraint_size, var_size);
        std::vector<double>lowerBound(constraint_size, 0.0);
        std::vector<double>upperBound(constraint_size, 0.0);
        long temp0 = 0;
        long temp1 = 0;
        long temp2 = 0;

        // Eq constraints
        for(long i=0; i<N-1; ++i)
        {
            temp0 = i+N;
            temp1 = i+2*N;
            constraint_matrix(i,temp0) = -1.0;
            constraint_matrix(i,temp0+1) = 1.0;
            constraint_matrix(i,temp1) = -delta_t/2.0;
            constraint_matrix(i,temp1+1) = -delta_t/2.0;
        }

        for(long i=N-1; i<2*N-2; ++i){
            temp0 = i-N+1;
            temp1 = i+1;
            temp2 = i+N+1;
            constraint_matrix(i,temp0) = -1.0;
            constraint_matrix(i,temp0+1) = 1.0;

            constraint_matrix(i,temp1) = -delta_t;

            constraint_matrix(i,temp2) = -delta_t*delta_t/3.0;
            constraint_matrix(i,temp2+1) = -delta_t*delta_t/6.0;
        }

        constraint_matrix(2*N-2,0) = 1.0;
        constraint_matrix(2*N-1,N) = 1.0;
        constraint_matrix(2*N,2*N) = 1.0;

        //initial conditions
        lowerBound[2*N-1] = initial_vel;
        upperBound[2*N-1] = initial_vel;
        lowerBound[2*N] = initial_acc;
        upperBound[2*N] = initial_acc;

        //inequal cons
        for(long i=0;i<var_size;i++){
            temp0 = i+size_cons_part1;
            constraint_matrix(temp0,i) = 1.0;
        }

        for(long i=var_size; i<size_cons_part2;i++){
            temp0 = i+size_cons_part1;
            temp1 = i-N;
            constraint_matrix(temp0,temp1) = -1.0;
            constraint_matrix(temp0,temp1+1) = 1.0;
        }

        for(long i=0; i<N;i++){
            temp0 = i+size_cons_part1;
            lowerBound[temp0] = min_s[i];
            upperBound[temp0] = max_s[i];
        }

        for(long i= N; i<2*N; i++){
            temp0 = i+size_cons_part1;
            temp1 = temp0 + N;
            temp2 = temp1 + N -1;

            lowerBound[temp0] = vmin;
            upperBound[temp0] = vmax;

            lowerBound[temp1] = amin;
            upperBound[temp1] = amax;

            lowerBound[temp2] = jmin*delta_t;
            upperBound[temp2] = jmax*delta_t;
        }

        

        // solve the QP problem
        const auto result = qp_solver_.optimize(hessian, constraint_matrix, gradient, lowerBound, upperBound);

        const std::vector<double> optval = std::get<0>(result);
        const int state_qp = std::get<2>(result);
        const int value_qp = std::get<3>(result);

        output.resize(N);
        for(unsigned int i=0; i<N; ++i)
        {
            output.position[i] = optval[i];
            output.velocity[i] = optval[i+N];
            output.acceleration[i] = optval[i+2*N];
        }

        for(unsigned int i=0; i<N-1; ++i)
        {
            double a_current = output.acceleration[i];
            double a_next    = output.acceleration[i+1];
            output.jerk[i] = (a_next - a_current)/ delta_t;
        }
        output.jerk[N-1] = output.jerk[N-2];

        return true;
    }

    bool QPSolver::solveHard(const double& initial_vel,
                             const double& initial_acc,
                             const double& ds,
                             const std::vector<double>& ref_vels,
                             const std::vector<double>& max_vels,
                             OutputInfo& output)
    {
        std::cerr << "This has the same form of the LPSolver::SolveHard" << std::endl;
        return false;
    }

    bool QPSolver::solveSoftPseudo(const double& initial_vel,
                                   const double& initial_acc,
                                   const double& ds,
                                   const std::vector<double>& ref_vels,
                                   const std::vector<double>& max_vels,
                                   OutputInfo& output)
    {
        /*
         * x = [b0, b1, ..., bN, |  a0, a1, ..., aN, | delta0, delta1, ..., deltaN, | sigma0, sigma1, ..., sigmaN] in R^{4N}
         * b: velocity^2
         * a: acceleration
         * delta: 0 < bi < vmax^2 + delta
         * sigma: amin < ai - sigma < amax
         */

        // Create Solver
        osqp::OSQPInterface qp_solver_;
        qp_solver_.updateMaxIter(20000);
        qp_solver_.updateRhoInterval(0);  // 0 means automatic
        qp_solver_.updateEpsRel(1.0e-4);  // def: 1.0e-4
        qp_solver_.updateEpsAbs(1.0e-8);  // def: 1.0e-4
        qp_solver_.updateVerbose(false);

        long N = static_cast<long>(ref_vels.size());
        const long var_size = 4*N;
        const long constraint_size = 3*N + 1;
        const double amax = param_.max_accel;
        const double amin = param_.min_decel;
        const double over_v_weight = param_.over_v_weight;
        const double over_a_weight = param_.over_a_weight;
        const double smooth_weight = param_.smooth_weight;

        // Hessian
        Eigen::MatrixXd hessian = Eigen::MatrixXd::Zero(var_size, var_size);
        for(int i=0; i<N; ++i)
        {
            hessian(i+2*N, i+2*N) = over_v_weight;
            hessian(i+3*N, i+3*N) = over_a_weight;
            if(i<N-1)
            {
                hessian(i+N, i+N) += smooth_weight/ds;
                hessian(i+1+N, i+1+N) += smooth_weight/ds;
                hessian(i+N, i+1+N) += -smooth_weight/ds;
                hessian(i+1+N, i+N) += -smooth_weight/ds;
            }
        }

        // Gradient
        std::vector<double> gradient(var_size, 0.0);
        for(long i=0; i<N; ++i)
        {
            const double v_max = std::max(max_vels.at(i), 0.1);
            gradient[i] = -1.0/(v_max*v_max);
        }

        // Constraint Matrix
        Eigen::MatrixXd constraint_matrix = Eigen::MatrixXd::Zero(constraint_size, var_size);
        std::vector<double>lowerBound(constraint_size, 0.0);
        std::vector<double>upperBound(constraint_size, 0.0);
        long constraint_num = 0;

        // 1. Velocity Constraint
        for(long i=0; i<N; ++i, ++constraint_num)
        {
            constraint_matrix(constraint_num, i) = 1.0; //b[i]
            constraint_matrix(constraint_num, i+2*N) = -1.0; // -delta[i]
            lowerBound[constraint_num] = 0.0;
            upperBound[constraint_num] = max_vels[i] * max_vels[i];
        }

        // 2. Acceleration Constraint
        for(long i=0; i<N; ++i, ++constraint_num)
        {
            constraint_matrix(constraint_num, i+N) = 1.0; //a[i]
            constraint_matrix(constraint_num, i+3*N) = -1.0; // -sigma[i]
            lowerBound[constraint_num] = amin;
            upperBound[constraint_num] = amax;
        }

        //3. Dynamic Constraint
        for(long i=0; i<N-1; ++i, ++constraint_num)
        {
            constraint_matrix(constraint_num, i) = -1.0; // -b[i]
            constraint_matrix(constraint_num, i+1) = 1.0; // b[i+1]
            constraint_matrix(constraint_num, i+N) = -2.0*ds; // a[i] * -2.0 * ds
            lowerBound[constraint_num] = 0.0;
            upperBound[constraint_num] = 0.0;
        }

        //5. Initial Condition
        constraint_matrix(constraint_num, 0) = 1.0;
        lowerBound[constraint_num] = initial_vel*initial_vel;
        upperBound[constraint_num] = initial_vel*initial_vel;
        ++constraint_num;
        constraint_matrix(constraint_num, N) = 1.0;
        lowerBound[constraint_num] = initial_acc;
        upperBound[constraint_num] = initial_acc;
        ++constraint_num;
        assert(constraint_num == constraint_size);

        // solve the QP problem
        const auto result = qp_solver_.optimize(hessian, constraint_matrix, gradient, lowerBound, upperBound);

        const std::vector<double> optval = std::get<0>(result);

        output.resize(N);
        for(unsigned int i=0; i<N; ++i)
        {
            output.velocity[i] = std::sqrt(std::max(optval[i], 0.0));
            output.acceleration[i] = optval[i+N];
        }

        for(unsigned int i=0; i<N-1; ++i)
        {
            double a_current = optval[i+N];
            double a_next    = optval[i+N+1];
            output.jerk[i] = (a_next - a_current) * output.velocity[i] / ds;
        }
        output.jerk[N-1] = output.jerk[N-2];

        return true;
    }

    bool QPSolver::solveHardPseudo(const double& initial_vel,
                                   const double& initial_acc,
                                   const double& ds,
                                   const std::vector<double>& ref_vels,
                                   const std::vector<double>& max_vels,
                                   OutputInfo& output)
    {
        /*
         * x = [b0, b1, ..., bN, |  a0, a1, ..., aN] in R^{2N}
         * b: velocity^2
         * a: acceleration
         * sigma: amin < ai < amax
         */

        // Create Solver
        osqp::OSQPInterface qp_solver_;
        qp_solver_.updateMaxIter(40000);
        qp_solver_.updateRhoInterval(0);  // 0 means automatic
        qp_solver_.updateEpsRel(1.0e-4);  // def: 1.0e-4
        qp_solver_.updateEpsAbs(1.0e-8);  // def: 1.0e-4
        qp_solver_.updateVerbose(false);

        long N = static_cast<long>(ref_vels.size());
        const long var_size = 2*N;
        const long constraint_size = 3*N + 1;
        const double amax = param_.max_accel;
        const double amin = param_.min_decel;
        const double smooth_weight = param_.smooth_weight;

        // Hessian
        Eigen::MatrixXd hessian = Eigen::MatrixXd::Zero(var_size, var_size);
        for(int i=0; i<N-1; ++i)
        {
            hessian(i+N, i+N) += smooth_weight/ds;
            hessian(i+1+N, i+1+N) += smooth_weight/ds;
            hessian(i+N, i+1+N) += -smooth_weight/ds;
            hessian(i+1+N, i+N) += -smooth_weight/ds;
        }

        // Gradient
        std::vector<double> gradient(var_size, 0.0);
        for(long i=0; i<N; ++i)
        {
            const double v_max = std::max(max_vels.at(i), 0.1);
            gradient[i] = -1.0/(v_max*v_max);
        }

        // Constraint Matrix
        Eigen::MatrixXd constraint_matrix = Eigen::MatrixXd::Zero(constraint_size, var_size);
        std::vector<double>lowerBound(constraint_size, 0.0);
        std::vector<double>upperBound(constraint_size, 0.0);
        long constraint_num = 0;

        // 1. Velocity Constraint
        for(long i=0; i<N; ++i, ++constraint_num)
        {
            constraint_matrix(constraint_num, i) = 1.0; //b[i]
            lowerBound[constraint_num] = 0.0;
            upperBound[constraint_num] = max_vels[i] * max_vels[i];
        }

        // 2. Acceleration Constraint
        for(long i=0; i<N; ++i, ++constraint_num)
        {
            constraint_matrix(constraint_num, i+N) = 1.0; //a[i]
            lowerBound[constraint_num] = amin;
            upperBound[constraint_num] = amax;
        }

        //3. Dynamic Constraint
        for(long i=0; i<N-1; ++i, ++constraint_num)
        {
            constraint_matrix(constraint_num, i) = -1.0; // -b[i]
            constraint_matrix(constraint_num, i+1) = 1.0; // b[i+1]
            constraint_matrix(constraint_num, i+N) = -2.0*ds; // a[i] * -2.0 * ds
            lowerBound[constraint_num] = 0.0;
            upperBound[constraint_num] = 0.0;
        }

        //5. Initial Condition
        constraint_matrix(constraint_num, 0) = 1.0;
        lowerBound[constraint_num] = initial_vel*initial_vel;
        upperBound[constraint_num] = initial_vel*initial_vel;
        ++constraint_num;
        constraint_matrix(constraint_num, N) = 1.0;
        lowerBound[constraint_num] = initial_acc;
        upperBound[constraint_num] = initial_acc;
        ++constraint_num;
        assert(constraint_num == constraint_size);

        // solve the QP problem
        const auto result = qp_solver_.optimize(hessian, constraint_matrix, gradient, lowerBound, upperBound);

        const std::vector<double> optval = std::get<0>(result);

        output.resize(N);
        for(unsigned int i=0; i<N; ++i)
        {
            output.velocity[i] = std::sqrt(std::max(optval[i], 0.0));
            output.acceleration[i] = optval[i+N];
        }

        for(unsigned int i=0; i<N-1; ++i)
        {
            double a_current = optval[i+N];
            double a_next    = optval[i+N+1];
            output.jerk[i] = (a_next - a_current) * output.velocity[i] / ds;
        }
        output.jerk[N-1] = output.jerk[N-2];

        return true;
    }
}
