#include "solver/qp_solver_osqp.h"

namespace osqp
{
    bool QPSolver::solvelat(const double& initial_vel,
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
        const long size_cons_part1 = 2*N+3;
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
        const double over_j_weight = param_.over_j_weight;

        const double ref_v = param_.ref_v; //final_v
        
        const double delta_t = dt;
        const double delta_t2 = dt*dt;

        // Hessian
        Eigen::MatrixXd hessian = Eigen::MatrixXd::Zero(var_size, var_size);
        for(int i=0; i<N; ++i)
        {
            // no 2 and gradient with 2 !!!! check
            hessian(i,i) = 2*over_s_weight;
            hessian(i+N, i+N) = 2*over_v_weight;
            hessian(i+2*N, i+2*N) = 2*over_a_weight + 2*over_j_weight/delta_t2;
        }

        for(int i=2*N+1; i<3*N-1; ++i){
            hessian(i-1,i) = -2*over_j_weight/delta_t2;
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

        //initial conditions
        constraint_matrix(2*N-2,0) = 1.0;
        constraint_matrix(2*N-1,N) = 1.0;
        constraint_matrix(2*N,2*N) = 1.0;

        
        lowerBound[2*N-1] = initial_vel;
        upperBound[2*N-1] = initial_vel;
        lowerBound[2*N] = initial_acc;
        upperBound[2*N] = initial_acc;

        //final conditions
        constraint_matrix(2*N+1,2*N-1) = 1.0; // final velocity;
        constraint_matrix(2*N+2,3*N-1) = 1.0; // final acceleration

        lowerBound[2*N+1] = ref_v;
        upperBound[2*N+1] = ref_v;
        lowerBound[2*N+2] = 0.0;
        upperBound[2*N+2] = 0.0;


        // ref_speed should be soft constraints ? or the final hard constraints

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

        // compute time
        for (unsigned int i=0; i<N; ++i){
            output.time[i] = i*delta_t;
        }

        return true;
    }



    bool QPSolver::solvelon(const double& initial_vel,
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
        const long size_cons_part1 = 2*N+3;
        const long size_cons_part2 = 4*N-1;
        const long constraint_size = size_cons_part1 + size_cons_part2;
        const double amax = param_.max_accel;
        const double amin = param_.min_decel;
        const double vmax = param_.max_vel;
        const double vmin = param_.min_vel;
        const double jmax = param_.max_jerk;
        const double jmin = param_.min_jerk;

        //const double over_s_weight = param_.over_s_weight;  // no smooth in speed planning
        const double over_v_weight = param_.over_v_weight;
        const double over_a_weight = param_.over_a_weight;
        const double over_j_weight = param_.over_j_weight;
        const double over_sref_weight = param_.over_sref_weight;
        const double over_vref_weight = param_.over_vref_weight;
        
        
        const double over_vend_weight = param_.over_vend_weight;
        const double over_aend_weight = param_.over_aend_weight;
        

        const double ref_v = param_.ref_v; //final_v
        
        const double delta_t = dt;
        const double delta_t2 = dt*dt;

        // Hessian
        Eigen::MatrixXd hessian_lowtri = Eigen::MatrixXd::Zero(var_size, var_size);
        for(int i=0; i<N; ++i)
        {
            // no 2 and gradient with 2 !!!! check
            hessian_lowtri(i,i) = 2*over_sref_weight;
            hessian_lowtri(i+N, i+N) = 2*over_v_weight + 2*over_vref_weight;
            //hessian_lowtri(i+2*N, i+2*N) = 2*over_a_weight + 2*over_j_weight/delta_t2;
        }

        hessian_lowtri(2*N-1,2*N-1) = hessian_lowtri(2*N-1, 2*N-1) + 2*over_vend_weight;
        

        for(int i=2*N+1; i<3*N-1; ++i){
            hessian_lowtri(i-1,i) = -2*over_j_weight/delta_t2;
            hessian_lowtri(i,i) = 2*over_a_weight + 4*over_j_weight/delta_t2;
        }

        hessian_lowtri(2*N,2*N) = 2*over_a_weight + 2*over_j_weight/delta_t2;
        hessian_lowtri(3*N-1,3*N-1) = 2*over_a_weight + 2*over_j_weight/delta_t2 + 2*over_aend_weight;

        Eigen::MatrixXd hessian =  hessian_lowtri;
        

        // Gradient
        std::vector<double> gradient(var_size, 0.0);
        for(long i=0; i<N; ++i)
        {
            gradient[i] = -2*ref_s[i]*over_sref_weight;
            gradient[i+N] = -2*ref_v*over_vref_weight;
            // note ref_a == 0.0;
        }

        gradient[2*N-1] = gradient[2*N-1] - 2*ref_v*over_vend_weight;
        //gradient[3*N-1] = 0; // ref_a == 0

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

        //initial conditions
        constraint_matrix(2*N-2,0) = 1.0;
        constraint_matrix(2*N-1,N) = 1.0;
        constraint_matrix(2*N,2*N) = 1.0;

        
        lowerBound[2*N-1] = initial_vel;
        upperBound[2*N-1] = initial_vel;
        lowerBound[2*N] = initial_acc;
        upperBound[2*N] = initial_acc;

        //final conditions
        constraint_matrix(2*N+1,2*N-1) = 1.0; // final velocity;
        constraint_matrix(2*N+2,3*N-1) = 1.0; // final acceleration

        lowerBound[2*N+1] = ref_v;
        upperBound[2*N+1] = ref_v;
        lowerBound[2*N+2] = 0.0;
        upperBound[2*N+2] = 0.0;


        // ref_speed should be soft constraints ? or the final hard constraints

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

        // compute time
        for (unsigned int i=0; i<N; ++i){
            output.time[i] = i*delta_t;
        }

        return true;
    }

    

    
}
