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
#include <ctime>
#include <casadi/casadi.hpp>

using namespace casadi;
using namespace std;

Eigen::MatrixXd NonlinearModelPredictiveControl::solve(const Vector6d_t& in_cart_velocities,
                                                       const JointStates& joint_states)
{
    // Declare variables
    SX u = SX::sym("u", 7); // control
    SX x = SX::sym("x", 7); // states


    // Number of differential states
    int nx = x.size1();

    // Number of controls
    int nu = u.size1();

    // Bounds and initial guess for the control
    vector<double> u_min =  { -2, -2, -2, -2, -2, -2, -2 };
    vector<double> u_max  = {  2,  2,  2,  2,  2,  2,  2 };
    vector<double> u_init = {  0.0,0.0,0.0,0.0,0.0,0.0,0.0  };

    // Bounds and initial guess for the state
    vector<double> x0_min = { joint_states.current_q_.data(0), joint_states.current_q_.data(1), joint_states.current_q_.data(2), joint_states.current_q_.data(3), joint_states.current_q_.data(4), joint_states.current_q_.data(5), joint_states.current_q_.data(6) };
    vector<double> x0_max = { joint_states.current_q_.data(0), joint_states.current_q_.data(1), joint_states.current_q_.data(2), joint_states.current_q_.data(3), joint_states.current_q_.data(4), joint_states.current_q_.data(5), joint_states.current_q_.data(6) };
    vector<double> x_min  = {-4.712, -1.92, -2.967, -2.618, -2.967, -2.443, -2.967};
    vector<double> x_max  = { 4.712,  1.92,  2.967,  2.618,  2.967,  2.443,  2.967};
    vector<double> xf_min = { -inf,-inf,-inf,-inf,-inf,-inf,-inf};
    vector<double> xf_max = {  inf, inf, inf, inf, inf, inf, inf };
    vector<double> x_init = { joint_states.current_q_.data(0), joint_states.current_q_.data(1), joint_states.current_q_.data(2), joint_states.current_q_.data(3), joint_states.current_q_.data(4), joint_states.current_q_.data(5), joint_states.current_q_.data(6) };

    // Final time
    double tf = 0.5;

    // Number of shooting nodes
    int ns = 5;

    // ODE right hand side and quadrature
    SX ode = SX::vertcat({u(0), u(1), u(2), u(3), u(4), u(5), u(6)});

    SX fk =  SX::horzcat({(1493.0*sin(x(3))*(sin(x(0))*sin(x(2)) - cos(x(0))*cos(x(1))*cos(x(2))))/5000.0 - (567.0*cos(x(0))*sin(x(1)))/1250.0 - (1493.0*cos(x(0))*cos(x(3))*sin(x(1)))/5000.0,
                     -(567.0*sin(x(0))*sin(x(1)))/1250.0 - (1493.0*sin(x(3))*(cos(x(0))*sin(x(2)) + cos(x(1))*cos(x(2))*sin(x(0))))/5000.0 - (1493.0*cos(x(3))*sin(x(0))*sin(x(1)))/5000.0,
                     (567.0*cos(x(1)))/1250.0 + (1493.0*cos(x(1))*cos(x(3)))/5000.0 - (1493.0*cos(x(2))*sin(x(1))*sin(x(3)))/5000.0 + 563.0/2000.0});

    SX x_desired = SX::horzcat({0.1, 0.1, 0.7});

    SX quad =  dot((fk - x_desired),(fk - x_desired));
    SXDict dae = {{"x", x}, {"p", u}, {"ode", ode}, {"quad", quad}};

    // Create an integrator (CVodes)
    Function F = integrator("integrator", "cvodes", dae, {{"t0", 0}, {"tf", tf/ns}});
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
        v_init.insert(v_init.end(), u_init.begin(), u_init.end());
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
    for(int k=0; k<ns; ++k){
      // Create an evaluation node
      MXDict I_out = F(MXDict{{"x0", X[k]}, {"p", U[k]}});

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
//    opts["ipopt.linear_solver"] = "ma27";

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

//    // Get the optimal state trajectory
//    vector<double> r_opt(ns+1), s_opt(ns+1);
//    for(int i=0; i<=ns; ++i){
//      r_opt[i] = V_opt.at(i*(nx+1));
//      s_opt[i] = V_opt.at(1+i*(nx+1));
//    }
//    cout << "r_opt = " << endl << r_opt << endl;
//    cout << "s_opt = " << endl << s_opt << endl;

    // Get the optimal control
    Eigen::VectorXd q_dot = Eigen::VectorXd::Zero(nu);

    for(int i=0; i<1; ++i)  // Copy only the first optimal control sequence
    {
        for(int j=0; j<nu; ++j)
        {
            q_dot[j] = V_opt.at(nx + j);
        }
    }

//    vector<double> u_opt(ns);
//    for(int i=0; i<ns; ++i){
//        q_dot << V_opt.at(nx + i*(nx+1));
//    }

    ROS_INFO_STREAM("U_opt: " << q_dot);

    return q_dot;
}

Eigen::MatrixXd NonlinearModelPredictiveControl::calculateWeighting(const Vector6d_t& in_cart_velocities, const JointStates& joint_states) const
{
    uint32_t cols = this->jacobian_data_.cols();
    Eigen::VectorXd weighting = Eigen::VectorXd::Ones(cols);
    return weighting.asDiagonal();
}
