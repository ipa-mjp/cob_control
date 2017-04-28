/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2017 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: care-o-bot
 * \note
 *   ROS stack name: cob_control
 * \note
 *   ROS package name: cob_twist_controller
 *
 * \author
 *   Author: Christoph Mark, email: christoph.mark@ipa.fraunhofer.de
 *
 * \date Date of creation: April, 2017
 *
 * \brief
 *   This header contains the interface description of constraint solvers
 *   Pure virtual methods have to be implemented in subclasses
 *
 ****************************************************************/

#include "cob_twist_controller/constraint_solvers/solvers/nonlinear_model_predictive_control.h"


// CONSTRUCT THE INTEGRATOR
Function NonlinearModelPredictiveControl::create_integrator(const unsigned int state_dim, const unsigned int control_dim, const double T,
                                                            const unsigned int N, SX ode, SX x, SX u, SX L)
{
    // Euler discretize
    double dt = T/((double)N);

    Function f = Function("f", {x, u}, {ode, L});

    MX X0 = MX::sym("X0", state_dim);
    MX U_ = MX::sym("U",control_dim);
    MX X_ = X0;
    MX Q = 0;

    vector<MX> input(2);
    input[0] = X_;
    input[1] = U_;
    MX qdot_new = f(input).at(0);
    MX Q_new = f(input).at(1);

    X_= X_+ dt * qdot_new;
    Q = Q + dt * Q_new;

    return Function("F", {X0, U_}, {X_, Q}, {"x0","p"}, {"xf", "qf"});
}


Eigen::MatrixXd NonlinearModelPredictiveControl::solve(const Vector6d_t& in_cart_velocities,
                                                       const JointStates& joint_states)
{
    // Final time
    double tf = 0.8;

    // Number of shooting nodes
    unsigned int ns = 8;

    unsigned int state_dim = 7;
    unsigned int control_dim = 7;

    // Distance to obstacle
    double min_dist = 0.5;

    // Declare variables
    SX u = SX::sym("u", state_dim); // control
    SX x = SX::sym("x", control_dim); // states

    int nx = x.size1();
    int nu = u.size1();

    // Bounds and initial guess for the control
    vector<double> u_min =  { -1, -1, -1, -1, -1, -1, -1 };
    vector<double> u_max  = {  1,  1,  1,  1,  1,  1,  1 };
    //vector<double> u_init = {  0.0,0.0,0.0,0.0,0.0,0.0,0.0  };

    // Bounds and initial guess for the state
    vector<double> x0_min = { joint_states.current_q_.data(0), joint_states.current_q_.data(1), joint_states.current_q_.data(2), joint_states.current_q_.data(3), joint_states.current_q_.data(4), joint_states.current_q_.data(5), joint_states.current_q_.data(6) };
    vector<double> x0_max = { joint_states.current_q_.data(0), joint_states.current_q_.data(1), joint_states.current_q_.data(2), joint_states.current_q_.data(3), joint_states.current_q_.data(4), joint_states.current_q_.data(5), joint_states.current_q_.data(6) };
    vector<double> x_min  = {-4.712, -1.92, -2.967, -2.618, -2.967, -2.443, -2.967};
    vector<double> x_max  = { 4.712,  1.92,  2.967,  2.618,  2.967,  2.443,  2.967};
    vector<double> xf_min = { -inf,-inf,-inf,-inf,-inf,-inf,-inf};
    vector<double> xf_max = {  inf, inf, inf, inf, inf, inf, inf };
    vector<double> x_init = { joint_states.current_q_.data(0), joint_states.current_q_.data(1), joint_states.current_q_.data(2), joint_states.current_q_.data(3), joint_states.current_q_.data(4), joint_states.current_q_.data(5), joint_states.current_q_.data(6) };



    // ODE right hand side and quadrature
    SX qdot = SX::vertcat({u(0), u(1), u(2), u(3), u(4), u(5), u(6)});

    // Position vector
    SX fk =  SX::horzcat({(1493.0*sin(x(3))*(sin(x(0))*sin(x(2)) - cos(x(0))*cos(x(1))*cos(x(2))))/5000.0 - (567.0*cos(x(0))*sin(x(1)))/1250.0 - (1493.0*cos(x(0))*cos(x(3))*sin(x(1)))/5000.0,
                     -(567.0*sin(x(0))*sin(x(1)))/1250.0 - (1493.0*sin(x(3))*(cos(x(0))*sin(x(2)) + cos(x(1))*cos(x(2))*sin(x(0))))/5000.0 - (1493.0*cos(x(3))*sin(x(0))*sin(x(1)))/5000.0,
                     (567.0*cos(x(1)))/1250.0 + (1493.0*cos(x(1))*cos(x(3)))/5000.0 - (1493.0*cos(x(2))*sin(x(1))*sin(x(3)))/5000.0 + 563.0/2000.0});

    // Rotational Matrix
    SX r31 = cos(x(6))*(cos(x(5))*(cos(x(4))*(cos(x(1))*sin(x(3)) + cos(x(2))*cos(x(3))*sin(x(1))) - sin(x(1))*sin(x(2))*sin(x(4))) + sin(x(5))*(cos(x(1))*cos(x(3)) - cos(x(2))*sin(x(1))*sin(x(3)))) - sin(x(6))*(sin(x(4))*(cos(x(1))*sin(x(3)) + cos(x(2))*cos(x(3))*sin(x(1))) + cos(x(4))*sin(x(1))*sin(x(2)));
    SX r11 = cos(x(6))*(sin(x(5))*(sin(x(3))*(sin(x(0))*sin(x(2)) - cos(x(0))*cos(x(1))*cos(x(2))) - cos(x(0))*cos(x(3))*sin(x(1))) - cos(x(5))*(cos(x(4))*(cos(x(3))*(sin(x(0))*sin(x(2)) - cos(x(0))*cos(x(1))*cos(x(2))) + cos(x(0))*sin(x(1))*sin(x(3))) + sin(x(4))*(cos(x(2))*sin(x(0)) + cos(x(0))*cos(x(1))*sin(x(2))))) + sin(x(6))*(sin(x(4))*(cos(x(3))*(sin(x(0))*sin(x(2)) - cos(x(0))*cos(x(1))*cos(x(2))) + cos(x(0))*sin(x(1))*sin(x(3))) - cos(x(4))*(cos(x(2))*sin(x(0)) + cos(x(0))*cos(x(1))*sin(x(2))));
    SX r21 = -cos(x(6))*(sin(x(5))*(sin(x(3))*(cos(x(0))*sin(x(2)) + cos(x(1))*cos(x(2))*sin(x(0))) + cos(x(3))*sin(x(0))*sin(x(1))) - cos(x(5))*(cos(x(4))*(cos(x(3))*(cos(x(0))*sin(x(2)) + cos(x(1))*cos(x(2))*sin(x(0))) - sin(x(0))*sin(x(1))*sin(x(3))) + sin(x(4))*(cos(x(0))*cos(x(2)) - cos(x(1))*sin(x(0))*sin(x(2))))) - sin(x(6))*(sin(x(4))*(cos(x(3))*(cos(x(0))*sin(x(2)) + cos(x(1))*cos(x(2))*sin(x(0))) - sin(x(0))*sin(x(1))*sin(x(3))) - cos(x(4))*(cos(x(0))*cos(x(2)) - cos(x(1))*sin(x(0))*sin(x(2))));
    SX r22 = sin(x(6))*(sin(x(5))*(sin(x(3))*(cos(x(0))*sin(x(2)) + cos(x(1))*cos(x(2))*sin(x(0))) + cos(x(3))*sin(x(0))*sin(x(1))) - cos(x(5))*(cos(x(4))*(cos(x(3))*(cos(x(0))*sin(x(2)) + cos(x(1))*cos(x(2))*sin(x(0))) - sin(x(0))*sin(x(1))*sin(x(3))) + sin(x(4))*(cos(x(0))*cos(x(2)) - cos(x(1))*sin(x(0))*sin(x(2))))) - cos(x(6))*(sin(x(4))*(cos(x(3))*(cos(x(0))*sin(x(2)) + cos(x(1))*cos(x(2))*sin(x(0))) - sin(x(0))*sin(x(1))*sin(x(3))) - cos(x(4))*(cos(x(0))*cos(x(2)) - cos(x(1))*sin(x(0))*sin(x(2))));
    SX r23 = - cos(x(5))*(sin(x(3))*(cos(x(0))*sin(x(2)) + cos(x(1))*cos(x(2))*sin(x(0))) + cos(x(3))*sin(x(0))*sin(x(1))) - sin(x(5))*(cos(x(4))*(cos(x(3))*(cos(x(0))*sin(x(2)) + cos(x(1))*cos(x(2))*sin(x(0))) - sin(x(0))*sin(x(1))*sin(x(3))) + sin(x(4))*(cos(x(0))*cos(x(2)) - cos(x(1))*sin(x(0))*sin(x(2))));

    // Transform to Euler angles
    SX x_e = atan2(-r31, r11);
    SX y_e = asin(r21);
    SX z_e = atan2(-r23,r22);
    SX x_rot = SX::horzcat({x_e, y_e, z_e});

    // Calculate squared distance
    SX dist = dot(fk,fk);

    // Barrier function for distance
//    SX barrier = exp((min_dist - sqrt(dist))/0.01);
    SX barrier = 1.0/(dist - min_dist*min_dist);

    // Desired endeffector pose
    SX x_desired = SX::horzcat({in_cart_velocities(0), in_cart_velocities(1), in_cart_velocities(2)});
    SX x_rot_desired = SX::horzcat({in_cart_velocities(3), in_cart_velocities(4), in_cart_velocities(5)});


    // Objective function
    SX L = dot((fk - x_desired), (fk - x_desired)) + dot((x_rot - x_rot_desired),(x_rot - x_rot_desired)) +
           dot(0.11*u, 0.11*u);


    Function F = create_integrator(nx, nu, tf, ns, qdot, x, u, L);

    // Total number of NLP variables
    int NV = nx*(ns+1) + nu*ns;

    // Declare variable vector for the NLP
    MX V = MX::sym("V",NV);

    // NLP variable bounds and initial guess
    vector<double> v_min,v_max,v_init;

    // Offset in V
    int offset=0;

    // State at each shooting node and control for each shooting interval
    vector<MX> X, U;
    for(int k=0; k<ns; ++k)
    {
        // Local state
        X.push_back( V.nz(Slice(offset,offset+nx)));
        if(k==0)
        {
            v_min.insert(v_min.end(), x0_min.begin(), x0_min.end());
            v_max.insert(v_max.end(), x0_max.begin(), x0_max.end());
        }
        else
        {
            v_min.insert(v_min.end(), x_min.begin(), x_min.end());
            v_max.insert(v_max.end(), x_max.begin(), x_max.end());
        }
        v_init.insert(v_init.end(), x_init.begin(), x_init.end());
        offset += nx;

        // Local control
        U.push_back( V.nz(Slice(offset,offset+nu)));
        v_min.insert(v_min.end(), u_min.begin(), u_min.end());
        v_max.insert(v_max.end(), u_max.begin(), u_max.end());
        v_init.insert(v_init.end(), u_init_.begin(), u_init_.end());
        offset += nu;
    }

    // State at end
    X.push_back(V.nz(Slice(offset,offset+nx)));
    v_min.insert(v_min.end(), xf_min.begin(), xf_min.end());
    v_max.insert(v_max.end(), xf_max.begin(), xf_max.end());
    v_init.insert(v_init.end(), x_init.begin(), x_init.end());
    offset += nx;

    // Make sure that the size of the variable vector is consistent with the number of variables that we have referenced
    casadi_assert(offset==NV);

    // Objective function
    MX J = 0;

    //Constraint function and bounds
    vector<MX> g;

    // Loop over shooting nodes
    for(int k=0; k<ns; ++k)
    {
        // Create an evaluation node
        MXDict I_out = F( MXDict{ {"x0", X[k]}, {"p", U[k]} });

        // Save continuity constraints
        g.push_back( I_out.at("xf") - X[k+1] );

        // Add objective function contribution
        J += I_out.at("qf");
    }

    // NLP
    MXDict nlp = {{"x", V}, {"f", J}, {"g", vertcat(g)}};

    // Set options
    Dict opts;

    opts["ipopt.tol"] = 1e-5;
    opts["ipopt.max_iter"] = 100;
//    opts["ipopt.hessian_approximation"] = "limited-memory";
//    opts["ipopt.linear_solver"] = "mumps";
    opts["ipopt.print_level"] = 0;
    opts["print_time"] = true;

    // Create an NLP solver and buffers
    Function solver = nlpsol("nlpsol", "ipopt", nlp, opts);

    std::map<std::string, DM> arg, res;

    // Bounds and initial guess
    arg["lbx"] = v_min;
    arg["ubx"] = v_max;
    arg["lbg"] = 0;
    arg["ubg"] = 0;
    arg["x0"] = v_init;

    // Solve the problem
    res = solver(arg);

    // Optimal solution of the NLP
    vector<double> V_opt(res.at("x"));
    vector<double> J_opt(res.at("f"));

    // Get the optimal control
    Eigen::VectorXd q_dot = Eigen::VectorXd::Zero(nu);

    for(int i=0; i<1; ++i)  // Copy only the first optimal control sequence
    {
        for(int j=0; j<nu; ++j)
        {
            q_dot[j] = V_opt.at(nx + j);
        }
    }

    return q_dot;
}

Eigen::MatrixXd NonlinearModelPredictiveControl::calculateWeighting(const Vector6d_t& in_cart_velocities, const JointStates& joint_states) const
{
    uint32_t cols = this->jacobian_data_.cols();
    Eigen::VectorXd weighting = Eigen::VectorXd::Ones(cols);
    return weighting.asDiagonal();
}
