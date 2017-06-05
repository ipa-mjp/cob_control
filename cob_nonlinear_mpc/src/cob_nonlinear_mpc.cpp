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
 *   ROS package name: cob_nonlinear_mpc
 *
 * \author
 *   Author: Bruno Brito  email: Bruno.Brito@ipa.fraunhofer.de
 *   Christoph Mark, email: Christoph.Mark@ipa.fraunhofer.de
 *
 * \date Date of creation: April, 2017
 *
 * \brief
 *
 *
 ****************************************************************/
#include <string>
#include <vector>
#include <limits>
#include <ros/ros.h>

#include <cob_nonlinear_mpc/cob_nonlinear_mpc.h>

#include <kdl_conversions/kdl_msg.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cob_srvs/SetString.h>
#include <string>
#include <Eigen/Dense>
#include <casadi/core/function/sx_function.hpp>
#include <stdlib.h>

bool CobNonlinearMPC::initialize()
{
    ros::NodeHandle nh_nmpc("nmpc");
    ros::NodeHandle nh_nmpc_dh("nmpc/dh");
    ros::NodeHandle nh_nmpc_base_dh("nmpc/base/dh");
    ros::NodeHandle nh_nmpc_constraints("nmpc/constraints");

    // JointNames
    if (!nh_.getParam("joint_names", joint_names))
    {
        ROS_ERROR("Parameter 'joint_names' not set");
        return false;
    }

    // nh_nmpc
    if (!nh_nmpc.getParam("transformations", transformation_names_))
    {
        ROS_ERROR("Parameter 'transformation_names' not set");
        return false;
    }
    int num_shooting_nodes;
    if (!nh_nmpc.getParam("shooting_nodes", num_shooting_nodes))
    {
        ROS_ERROR("Parameter 'num_shooting_nodes_' not set");
        return false;
    }
    double time_horizon;
    if (!nh_nmpc.getParam("time_horizon", time_horizon))
    {
        ROS_ERROR("Parameter 'time_horizon' not set");
        return false;
    }
    int state_dim;
    if (!nh_nmpc.getParam("state_dim", state_dim))
    {
        ROS_ERROR("Parameter 'state_dim' not set");
        return false;
    }
    int control_dim;
    if (!nh_nmpc.getParam("control_dim", control_dim))
    {
        ROS_ERROR("Parameter 'control_dim' not set");
        return false;
    }

    mpc_ctr_.reset(new MPC(num_shooting_nodes,time_horizon ,state_dim,control_dim));
    if (!nh_nmpc.getParam("base/base_active", robot_.base_active_))
    {
        ROS_ERROR("Parameter 'base/base_active' not set");
        return false;
    }

    // nh_nmpc_constraints
    vector<double> state_path_constraints_min;
    if (!nh_nmpc_constraints.getParam("state/path_constraints/min", state_path_constraints_min))
    {
        ROS_ERROR("Parameter 'state/path_constraints/min' not set");
        return false;
    }
    vector<double> state_path_constraints_max;
    if (!nh_nmpc_constraints.getParam("state/path_constraints/max", state_path_constraints_max))
    {
        ROS_ERROR("Parameter 'state/path_constraints/max' not set");
        return false;
    }
    mpc_ctr_->set_path_constraints(state_path_constraints_min,state_path_constraints_max);

    vector<double> state_terminal_constraints_min;
    if (!nh_nmpc_constraints.getParam("state/terminal_constraints/min", state_terminal_constraints_min))
    {
        ROS_ERROR("Parameter 'state/terminal_constraints/min' not set");
        return false;
    }
    vector<double> state_terminal_constraints_max;
    if (!nh_nmpc_constraints.getParam("state/terminal_constraints/max", state_terminal_constraints_max))
    {
        ROS_ERROR("Parameter 'state/terminal_constraints/max' not set");
        return false;
    }
    mpc_ctr_->set_state_constraints(state_terminal_constraints_min,state_terminal_constraints_max);
    vector<double> input_constraints_min;
    if (!nh_nmpc_constraints.getParam("input/input_constraints/min", input_constraints_min))
    {
        ROS_ERROR("Parameter 'input/input_constraints/min' not set");
        return false;
    }
    vector<double> input_constraints_max;
    if (!nh_nmpc_constraints.getParam("input/input_constraints/max", input_constraints_max))
    {
        ROS_ERROR("Parameter 'input/input_constraints/max' not set");
        return false;
    }
    mpc_ctr_->set_input_constraints(input_constraints_min,input_constraints_max);

    // Chain
    if (!nh_.getParam("chain_base_link", chain_base_link_))
    {
        ROS_ERROR("Parameter 'chain_base_link' not set");
        return false;
    }

    if (!nh_.getParam("chain_tip_link", chain_tip_link_))
    {
        ROS_ERROR("Parameter 'chain_tip_link' not set");
        return false;
    }

    this->process_KDL_tree();

    this->generate_symbolic_forward_kinematics();

    this->generate_bounding_volumes();

    vector<double> tmp;
    for(int k=0; k < mpc_ctr_->get_num_shooting_nodes(); ++k)
    {
        tmp.clear();
        for(int i=0; i < mpc_ctr_->get_control_dim(); ++i)
        {
            tmp.push_back(0);
        }
        u_open_loop_.push_back(tmp);
    }

    for(int k=1; k <= mpc_ctr_->get_num_shooting_nodes(); ++k)
    {
        tmp.clear();

        for(int i=0; i < mpc_ctr_->get_state_dim(); ++i)
        {
            tmp.push_back(0);
        }

        x_open_loop_.push_back(tmp);
    }

    for(int i = 0; i < mpc_ctr_->get_control_dim(); i++)
    {
        ROS_INFO_STREAM("CONTROL DIM: "<<mpc_ctr_->get_control_dim());
        u_init_.push_back(0);
    }

    joint_state_ = KDL::JntArray(robot_.kinematic_chain.getNrOfJoints());
    jointstate_sub_ = nh_.subscribe("joint_states", 1, &CobNonlinearMPC::jointstateCallback, this);

    if(robot_.base_active_){
        odometry_state_ = KDL::JntArray(3);
        odometry_sub_ = nh_.subscribe("base/odometry", 1, &CobNonlinearMPC::odometryCallback, this);
        base_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("base/command", 1);
    }

    frame_tracker_sub_ = nh_.subscribe("command_pose", 1, &CobNonlinearMPC::FrameTrackerCallback, this);
    pub_ = nh_.advertise<std_msgs::Float64MultiArray>("joint_group_velocity_controller/command", 1);

    ROS_WARN_STREAM(nh_.getNamespace() << "/NMPC...initialized!");
    return true;
}

void CobNonlinearMPC::FrameTrackerCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    KDL::JntArray state = getJointState();

    Eigen::MatrixXd qdot = mpc_step(*msg, state);

    geometry_msgs::Twist base_vel_msg;
    std_msgs::Float64MultiArray vel_msg;

    if(robot_.base_active_){
        base_vel_msg.linear.x = qdot(0);
        base_vel_msg.linear.y = qdot(1);
        base_vel_msg.linear.z = 0;
        base_vel_msg.angular.x = 0;
        base_vel_msg.angular.y = 0;
        base_vel_msg.angular.z = qdot(2);

        base_vel_pub_.publish(base_vel_msg);

        for (unsigned int i = 3; i < joint_state_.rows()+3; i++)
        {
            vel_msg.data.push_back(qdot(i));
        }
        pub_.publish(vel_msg);

    }
    else{
        for (unsigned int i = 0; i < joint_state_.rows(); i++)
        {
            vel_msg.data.push_back(qdot(i));
        }
        pub_.publish(vel_msg);
    }
}


void CobNonlinearMPC::jointstateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{

    KDL::JntArray q_temp = joint_state_;

    int count = 0;

    for (uint16_t j = 0; j < joint_state_.rows(); j++)
    {
        for (uint16_t i = 0; i < msg->name.size(); i++)
        {
            if (strcmp(msg->name[i].c_str(), joint_names[j].c_str()) == 0)
            {

                q_temp(j) = msg->position[i];
                count++;
                break;
            }
        }
    }
    joint_state_ = q_temp;
}


void CobNonlinearMPC::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    KDL::JntArray temp = odometry_state_;
    KDL::Frame odom_frame_bl;
    tf::StampedTransform odom_transform_bl;

    temp(0) = msg->pose.pose.position.x;
    temp(1) = msg->pose.pose.position.y;
    temp(2) = msg->pose.pose.orientation.z;

    odometry_state_ = temp;
}


KDL::JntArray CobNonlinearMPC::getJointState()
{
    KDL:: JntArray tmp(joint_state_.rows() + odometry_state_.rows());
//    KDL:: JntArray tmp(joint_state_.rows());

    ROS_INFO("STATE SIZE: %i Odometry State SIze %i", tmp.rows(), odometry_state_.rows());

    for(int i = 0; i < odometry_state_.rows(); i++)
    {
        tmp(i) = odometry_state_(i);
    }

    for(int i = 0 ; i < joint_state_.rows(); i++)
    {
        tmp(i+odometry_state_.rows()) = this->joint_state_(i);
    }

    return tmp;
}


Eigen::MatrixXd CobNonlinearMPC::mpc_step(const geometry_msgs::Pose pose,
                                          const KDL::JntArray& state)
{
    // Distance to obstacle
    double min_dist = 0.15;

    // Bounds and initial guess for the control
    ROS_INFO_STREAM("input_constraints_min_: " <<input_constraints_min_.size());
    ROS_INFO_STREAM("input_constraints_max_: " <<input_constraints_max_.size());
    vector<double> u_min =  input_constraints_min_;
    vector<double> u_max  = input_constraints_max_;

    ROS_INFO("Bounds and initial guess for the state");
    vector<double> x0_min;
    vector<double> x0_max;
    vector<double> x_init;
    ROS_INFO_STREAM("state rows: " <<state.rows());
    for(unsigned int i=0; i < state.rows();i++)
    {
        x0_min.push_back(state(i));
        x0_max.push_back(state(i));
        x_init.push_back(state(i));
    }

    vector<double> x_min  = state_path_constraints_min_;
    vector<double> x_max  = state_path_constraints_max_;
    vector<double> xf_min = state_terminal_constraints_min_;
    vector<double> xf_max = state_terminal_constraints_max_;

    ROS_INFO("ODE right hand side and quadrature");
    SX qdot = SX::vertcat({u_});

    ROS_INFO("Current Quaternion and Position Vector.");
    double kappa = 0.001; // Small regulation term for numerical stability for the NLP

    SX q_c = SX::vertcat({
        0.5 * sqrt(fk_(0,0) + fk_(1,1) + fk_(2,2) + 1.0 + kappa),
        0.5 * (sign((fk_(2,1) - fk_(1,2)))) * sqrt(fk_(0,0) - fk_(1,1) - fk_(2,2) + 1.0 + kappa),
        0.5 * (sign((fk_(0,2) - fk_(2,0)))) * sqrt(fk_(1,1) - fk_(2,2) - fk_(0,0) + 1.0 + kappa),
        0.5 * (sign((fk_(1,0) - fk_(0,1)))) * sqrt(fk_(2,2) - fk_(0,0) - fk_(1,1) + 1.0 + kappa)
    });

    SX p_c = SX::vertcat({fk_(0,3), fk_(1,3), fk_(2,3)});


    ROS_INFO("Desired Goal-pose");
    SX x_d = SX::vertcat({pose.position.x, pose.position.y, pose.position.z});
    SX q_d = SX::vertcat({pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z});

    // Prevent collision with Base_link
    SX barrier;
    SX dist;

    std::unordered_map<std::string, std::vector<std::string> >::iterator it_scm;

    int counter = 0;
    double bv_radius;

    for( it_scm = self_collision_map_.begin(); it_scm != self_collision_map_.end(); it_scm++)
    {
        std::vector<string> scm_collision_links = it_scm->second;
        for(int i=0; i<scm_collision_links.size(); i++)
        {
            ROS_WARN_STREAM(it_scm->first);
            vector<vector<SX>> p1_mat = bvh_matrix[it_scm->first];
            vector<vector<SX>> p2_mat = bvh_matrix[scm_collision_links.at(i)];

            for(int k=0; k<p1_mat.size(); k++)
            {
                if(it_scm->first == "body")
                {
                    bv_radius = bvb_radius_.at(k);
                }
                else
                {
                    bv_radius = 0.1;
                }

                vector<SX> p1_vec = p1_mat.at(k);
                for(int m=0; m<p2_mat.size(); m++)
                {
                    vector<SX> p2_vec = p2_mat.at(m);

                    SX p1 = SX::vertcat({p1_vec.at(0)});
                    SX p2 = SX::vertcat({p2_vec.at(0)});
                    dist = dot(p1 - p2, p1 - p2);

                    if(counter == 0)
                    {
                        barrier = exp((bv_radius - sqrt(dist))/0.01);
                        counter = 1;
                    }
                    else
                    {
                        barrier += exp((bv_radius - sqrt(dist))/0.01);
                    }
                }
            }
        }
    }

    // Get orientation error
    SX q_c_inverse = SX::vertcat({q_c(0), -q_c(1), -q_c(2), -q_c(3)});
    SX e_quat= quaternion_product(q_c_inverse,q_d);
    SX error_attitute = SX::vertcat({ e_quat(1), e_quat(2), e_quat(3)});

    // L2 norm of the control signal
    SX R = SX::scalar_matrix(mpc_ctr_->get_control_dim(),1,1);;
    SX energy = dot(sqrt(R)*u_,sqrt(R)*u_);

    // L2 norm of the states
    std::vector<int> state_convariance(mpc_ctr_->get_state_dim(),1);
    SX S = 0.01*SX::scalar_matrix(mpc_ctr_->get_state_dim(),1,1);
    SX motion = dot(sqrt(S)*x_,sqrt(S)*x_);

    // Objective
    SX L = 10*dot(p_c-x_d,p_c-x_d) ;//+ energy + 10 * dot(error_attitute,error_attitute) + barrier;

    // Create Euler integrator function
    Function F = create_integrator(mpc_ctr_->get_state_dim(), mpc_ctr_->get_control_dim(), mpc_ctr_->get_time_horizon(), mpc_ctr_->get_num_shooting_nodes(), qdot, x_, u_, L);

    // Total number of NLP variables
    int NV = mpc_ctr_->get_state_dim()*(mpc_ctr_->get_num_shooting_nodes()+1) + mpc_ctr_->get_control_dim()*mpc_ctr_->get_num_shooting_nodes();

    // Declare variable vector for the NLP
    MX V = MX::sym("V",NV);

    // NLP variable bounds and initial guess
    vector<double> v_min,v_max,v_init;

    // Offset in V
    int offset=0;

    // State at each shooting node and control for each shooting interval
    vector<MX> X, U;

    for(unsigned int k=0; k<mpc_ctr_->get_num_shooting_nodes(); ++k)
    {
        // Local state
        X.push_back( V.nz(Slice(offset,offset+mpc_ctr_->get_state_dim())));

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
        offset += mpc_ctr_->get_state_dim();

        // Local control via shift initialization
        U.push_back( V.nz(Slice(offset,offset+mpc_ctr_->get_control_dim())));
        v_min.insert(v_min.end(), u_min.begin(), u_min.end());
        v_max.insert(v_max.end(), u_max.begin(), u_max.end());

        v_init.insert(v_init.end(), u_init_.begin(), u_init_.end());
        offset += mpc_ctr_->get_control_dim();
    }

    // State at end
    X.push_back(V.nz(Slice(offset,offset+mpc_ctr_->get_state_dim())));
    v_min.insert(v_min.end(), xf_min.begin(), xf_min.end());
    v_max.insert(v_max.end(), xf_max.begin(), xf_max.end());
    v_init.insert(v_init.end(), u_init_.begin(), u_init_.end());
    offset += mpc_ctr_->get_state_dim();

    // Make sure that the size of the variable vector is consistent with the number of variables that we have referenced
    casadi_assert(offset==NV);

    // Objective function
    MX J = 0;

    //Constraint function and bounds
    vector<MX> g;

    // Loop over shooting nodes
    for(unsigned int k=0; k<mpc_ctr_->get_num_shooting_nodes(); ++k)
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

    opts["ipopt.tol"] = 1e-4;
    opts["ipopt.max_iter"] = 20;
//    opts["ipopt.hessian_approximation"] = "limited-memory";
//    opts["ipopt.hessian_constant"] = "yes";
    opts["ipopt.linear_solver"] = "ma27";
    opts["ipopt.print_level"] = 0;
    opts["print_time"] = true;
    opts["expand"] = true;  // Removes overhead

    ROS_INFO("Create an NLP solver and buffers");
    Function solver = nlpsol("nlpsol", "ipopt", nlp, opts);

    std::map<std::string, DM> arg, res;

    ROS_INFO("Bounds and initial guess");
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
    Eigen::VectorXd q_dot = Eigen::VectorXd::Zero(mpc_ctr_->get_state_dim());
//    Eigen::VectorXd x_new = Eigen::VectorXd::Zero(mpc_ctr_->get_state_dim());
    vector<double> x_new;
    SX sx_x_new;
    u_open_loop_.clear();
    x_open_loop_.clear();
    x_new.clear();

    for(int i=0; i<1; ++i)  // Copy only the first optimal control sequence
    {
        for(int j=0; j<mpc_ctr_->get_control_dim(); ++j)
        {
            q_dot[j] = V_opt.at(mpc_ctr_->get_state_dim() + j);
            x_new.push_back(V_opt.at(j));
        }
    }
    sx_x_new = SX::vertcat({x_new});

    sx_x_new = SX::vertcat({x_new});
    // Safe optimal control sequence at time t_k and take it as inital guess at t_k+1

    // Plot bounding volumes
    geometry_msgs::Point point;
    point.x = 0;
    point.y = 0;
    point.z = 0;

    SX result;
    for( it_scm = self_collision_map_.begin(); it_scm != self_collision_map_.end(); it_scm++)
    {
        vector<string> tmp = it_scm->second;

        for(int i=0; i<tmp.size();i++)
        {
            vector<vector<SX>> SX_vec = bvh_matrix[tmp.at(i)];
            for(int k=0; k<SX_vec.size(); k++)
            {
                SX test = SX::horzcat({SX_vec.at(k).at(0)});
                Function tesssst = Function("test", {x_}, {test});
                result = tesssst(sx_x_new).at(0);
                point.x = (double)result(0);
                point.y = (double)result(1);
                point.z = (double)result(2);

                bv_radius = 0.1;
                visualizeBVH(point, bv_radius, i+i*tmp.size());
            }
        }


        vector<vector<SX>> SX_vec = bvh_matrix[it_scm->first];
        for(int k=0; k<SX_vec.size(); k++)
        {
            SX test = SX::horzcat({SX_vec.at(k).at(0)});
            Function tesssst = Function("test", {x_}, {test});
            result = tesssst(sx_x_new).at(0);
            point.x = (double)result(0);
            point.y = (double)result(1);
            point.z = (double)result(2);

            if(it_scm->first == "body")
            {
                bv_radius = bvb_radius_.at(k);
            }
            else
            {
                bv_radius = 0.1;
            }
            visualizeBVH(point, bv_radius, k+tmp.size()+SX_vec.size());
        }
    }
    return q_dot;
}

Function CobNonlinearMPC::create_integrator(const unsigned int state_dim, const unsigned int control_dim, const double T,
                                            const unsigned int N, SX ode, SX x, SX u, SX L)
{
    // Euler discretize
    double dt = T/((double)N);

    Function f = Function("f", {x, u}, {ode, L});
//
//    f.generate("f");
//
//    // Compile the C-code to a shared library
//    string compile_command = "gcc -fPIC -shared -O3 f.c -o f.so";
//    int flag = system(compile_command.c_str());
//    casadi_assert_message(flag==0, "Compilation failed");
//
//    Function f_ext = external("f");

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

    Function F = Function("F", {X0, U_}, {X_, Q}, {"x0","p"}, {"xf", "qf"});
    return F.expand("F");   // Remove overhead
}


SX CobNonlinearMPC::dual_quaternion_product(SX q1, SX q2)
{
    SX q1_real = SX::vertcat({q1(0),q1(1),q1(2),q1(3)});
    SX q1_dual = SX::vertcat({q1(4),q1(5),q1(6),q1(7)});
    SX q2_real = SX::vertcat({q2(0),q2(1),q2(2),q2(3)});
    SX q2_dual = SX::vertcat({q2(4),q2(5),q2(6),q2(7)});

    SX q1q2_real = quaternion_product(q1_real,q2_real);
    SX q1_real_q2_dual = quaternion_product(q1_real,q2_dual);
    SX q1_dual_q2_real = quaternion_product(q1_dual,q2_real);

    SX q_prod = SX::vertcat({
        q1q2_real,
        q1_real_q2_dual + q1_dual_q2_real
    });

    return q_prod;
}

SX CobNonlinearMPC::quaternion_product(SX q1, SX q2)
{
    SX q1_v = SX::vertcat({q1(1),q1(2),q1(3)});
    SX q2_v = SX::vertcat({q2(1),q2(2),q2(3)});

    SX c = SX::cross(q1_v,q2_v);

    SX q_new = SX::vertcat({
        q1(0) * q2(0) - dot(q1_v,q2_v),
        q1(0) * q2(1) + q2(0) * q1(1) + c(0),
        q1(0) * q2(2) + q2(0) * q1(2) + c(1),
        q1(0) * q2(3) + q2(0) * q1(3) + c(2)
    });

    return q_new;
}


void CobNonlinearMPC::visualizeBVH(const geometry_msgs::Point point, double radius, int id)
{
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.lifetime = ros::Duration();
    marker.action = visualization_msgs::Marker::ADD;
    marker.ns = "preview";
    marker.header.frame_id = "odom_combined";


    marker.scale.x = 2*radius;
    marker.scale.y = 2*radius;
    marker.scale.z = 2*radius;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.1;

    marker_array_.markers.clear();

    marker.id = id;
    marker.pose.position.x = point.x;
    marker.pose.position.y = point.y;
    marker.pose.position.z = point.z;
    marker_array_.markers.push_back(marker);

    marker_pub_.publish(marker_array_);
}
