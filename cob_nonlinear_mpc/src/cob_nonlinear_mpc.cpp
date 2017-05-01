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
 *   Author: Christoph Mark, email: Christoph.Mark@ipa.fraunhofer.de
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

#include <Eigen/Dense>

bool CobNonlinearMPC::initialize()
{
    ros::NodeHandle nh_nmpc("nmpc");
    ros::NodeHandle nh_nmpc_dh("nmpc/dh");
    ros::NodeHandle nh_nmpc_constraints("nmpc/constraints");


    // nh_nmpc
    if (!nh_nmpc.getParam("transformations", transformation_names_))
    {
        ROS_ERROR("Parameter 'num_transformations' not set");
        return false;
    }

    if (!nh_nmpc.getParam("shooting_nodes", num_shooting_nodes_))
    {
        ROS_ERROR("Parameter 'num_shooting_nodes_' not set");
        return false;
    }

    if (!nh_nmpc.getParam("time_horizon", time_horizon_))
    {
        ROS_ERROR("Parameter 'time_horizon' not set");
        return false;
    }

    if (!nh_nmpc.getParam("state_dim", state_dim_))
    {
        ROS_ERROR("Parameter 'num_transformations' not set");
        return false;
    }
    if (!nh_nmpc.getParam("control_dim", control_dim_))
    {
        ROS_ERROR("Parameter 'num_transformations' not set");
        return false;
    }

    // nh_nmpc_constraints
    if (!nh_nmpc_constraints.getParam("state/path_constraints/min", state_path_constraints_min_))
    {
        ROS_ERROR("Parameter 'state/path_constraints/min' not set");
        return false;
    }
    if (!nh_nmpc_constraints.getParam("state/path_constraints/max", state_path_constraints_max_))
    {
        ROS_ERROR("Parameter 'state/path_constraints/max' not set");
        return false;
    }

    if (!nh_nmpc_constraints.getParam("state/terminal_constraints/min", state_terminal_constraints_min_))
    {
        ROS_ERROR("Parameter 'state/terminal_constraints/min' not set");
        return false;
    }
    if (!nh_nmpc_constraints.getParam("state/terminal_constraints/max", state_terminal_constraints_max_))
    {
        ROS_ERROR("Parameter 'state/terminal_constraints/max' not set");
        return false;
    }

    if (!nh_nmpc_constraints.getParam("input/input_constraints/min", input_constraints_min_))
    {
        ROS_ERROR("Parameter 'input/input_constraints/min' not set");
        return false;
    }
    if (!nh_nmpc_constraints.getParam("input/input_constraints/max", input_constraints_max_))
    {
        ROS_ERROR("Parameter 'input/input_constraints/max' not set");
        return false;
    }

    // nh_nmpc_dh
    for(unsigned int i = 0; i < transformation_names_.size(); i++)
    {
        DH param;

        std::string test = transformation_names_.at(i)+"/a";
        if (!nh_nmpc_dh.getParam(transformation_names_.at(i)+"/a", param.a))
        {
            ROS_ERROR("Parameter 'a' not set");
            return false;
        }
        if (!nh_nmpc_dh.getParam(transformation_names_.at(i)+"/d", param.d))
        {
            ROS_ERROR("Parameter 'd' not set");
            return false;
        }
        if (!nh_nmpc_dh.getParam(transformation_names_.at(i)+"/alpha", param.alpha))
        {
            ROS_ERROR("Parameter 'alpha' not set");
            return false;
        }
        if (!nh_nmpc_dh.getParam(transformation_names_.at(i)+"/theta", param.theta))
        {
            ROS_ERROR("Parameter 'theta' not set");
            return false;
        }

        dh_params.push_back(param);
    }

    // Casadi symbolics
    u_ = SX::sym("u", state_dim_);  // control
    x_ = SX::sym("x", control_dim_); // states

    for(int i=0; i< dh_params.size(); i++)
    {
        SX theta = x_(i);
        double d = dh_params.at(i).d;
        double a = dh_params.at(i).a;
        double alpha = dh_params.at(i).alpha;

        SX T = SX::sym("T",4,4);
        T(0,0) = cos(theta); T(0,1) = -sin(theta) * cos(alpha); T(0,2) = sin(theta) * sin(alpha) ; T(0,3) = a * cos(theta);
        T(1,0) = sin(theta); T(1,1) = cos(theta) * cos(alpha) ; T(1,2) = -cos(theta) * sin(alpha); T(1,3) = a * sin(theta);
        T(2,0) = 0         ; T(2,1) = sin(alpha)              ; T(2,2) = cos(alpha)              ; T(2,3) = d;
        T(3,0) = 0         ; T(3,1) = 0                       ; T(3,2) = 0                       ; T(3,3) = 1;

        transformation_vector.push_back(T);
    }

    for(int i=0; i< transformation_vector.size(); i++)
    {
        if(i==0)
        {
            fk_ = transformation_vector.at(i);
        }
        else
        {
            fk_ = mtimes(fk_,transformation_vector.at(i));
        }
    }




    joint_state_ = KDL::JntArray(7);
    odometry_state_ = KDL::JntArray(3);
    jointstate_sub_ = nh_.subscribe("joint_states", 1, &CobNonlinearMPC::jointstateCallback, this);
    odometry_sub_ = nh_.subscribe("base/odometry", 1, &CobNonlinearMPC::odometryCallback, this);
    pose_sub_ = nh_.subscribe("command_pose", 1, &CobNonlinearMPC::poseCallback, this);

    base_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("base/command", 1);
    pub_ = nh_.advertise<std_msgs::Float64MultiArray>("joint_group_velocity_controller/command", 1);

    ROS_INFO_STREAM(nh_.getNamespace() << "/NMPC...initialized!");
    return true;
}

void CobNonlinearMPC::poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    KDL::JntArray state = getJointState();

    Eigen::MatrixXd qdot = mpc_step(*msg, state);

//    geometry_msgs::Twist base_vel_msg;
    std_msgs::Float64MultiArray vel_msg;
//
//    base_vel_msg.linear.x = qdot(0);
//    base_vel_msg.linear.y = qdot(1);
//    base_vel_msg.linear.z = 0;
//    base_vel_msg.angular.x = 0;
//    base_vel_msg.angular.y = 0;
//    base_vel_msg.angular.z = qdot(2);
//
//
//    base_vel_pub_.publish(base_vel_msg);
//
//
//    for (unsigned int i = 3; i < 10; i++)
//    {
//        vel_msg.data.push_back(qdot(i));
//    }
//    pub_.publish(vel_msg);

        for (unsigned int i = 0; i < 7; i++)
        {
            vel_msg.data.push_back(qdot(i));
        }
        pub_.publish(vel_msg);
}


void CobNonlinearMPC::jointstateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{

    KDL::JntArray q_temp = joint_state_;
    std::vector<std::string> joint_names;
//    joint_names = {"arm_left_1_link", "arm_left_2_link", "arm_left_3_link", "arm_left_4_link", "arm_left_5_link", "arm_left_6_link", "arm_left_7_link"};
    joint_names = {"arm_1_joint","arm_2_joint","arm_3_joint","arm_4_joint","arm_5_joint","arm_6_joint","arm_7_joint"};

    int count = 0;

    for (uint16_t j = 0; j < 7; j++)
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
//    KDL:: JntArray tmp(joint_state_.rows() + odometry_state_.rows());
    KDL:: JntArray tmp(joint_state_.rows());

//    tmp = this->odometry_state_;

    for(int i = 0; i < joint_state_.rows(); i++)
    {
        tmp(i) = joint_state_(i);
    }

//    for(int i = joint_state_.rows() ; i < joint_state_.rows() + odometry_state_.rows(); i++)
//    {
//        tmp(i) = this->odometry_state_(i - joint_state_.rows());
//    }

    return tmp;
}


Eigen::MatrixXd CobNonlinearMPC::mpc_step(const geometry_msgs::Pose pose,
                                          const KDL::JntArray& state)
{
    // Distance to obstacle
    double min_dist = 0.7;

    // Bounds and initial guess for the control
    vector<double> u_min =  input_constraints_min_;
    vector<double> u_max  = input_constraints_max_;

    // Bounds and initial guess for the state
    vector<double> x0_min;
    vector<double> x0_max;
    vector<double> x_init;
    for(unsigned int i=0; i< state.rows();i++)
    {
        x0_min.push_back(state(i));
        x0_max.push_back(state(i));
        x_init.push_back(state(i));
    }
    vector<double> x_min  = state_path_constraints_min_;
    vector<double> x_max  = state_path_constraints_max_;
    vector<double> xf_min = state_terminal_constraints_min_;
    vector<double> xf_max = state_terminal_constraints_max_;

    // ODE right hand side and quadrature
    SX qdot = SX::vertcat({u_(0), u_(1), u_(2), u_(3), u_(4), u_(5), u_(6)});

    // Position vector of end-effector
    SX p = SX::horzcat({fk_(0,3), fk_(1,3), fk_(2,3)});


//    // Transform to Euler angles
    SX x_e = atan2(-fk_(2,0), fk_(0,0));
    SX y_e = asin(fk_(1,0));
    SX z_e = atan2(-fk_(1,2),fk_(1,1));
    SX x_rot = SX::horzcat({x_e, y_e, z_e});

//    // Quaternion
//    SX b = sqrt(1 + fk_(0,0) + fk_(1,1) + fk_(2,2));
//    SX qw = 0.5*b;
//    SX qx = (fk_(2,1) - fk_(1,2)) / (4*b);
//    SX qy = (fk_(0,2) - fk_(2,0)) / (4*b);
//    SX qz = (fk_(1,0) - fk_(0,1)) / (4*b);
//    SX q_fk = SX::horzcat({qw, qx, qy, qz});
//
//    tf::Quaternion quat_tf;
//    quat_tf.setW(pose.orientation.w);
//    quat_tf.setX(pose.orientation.x);
//    quat_tf.setY(pose.orientation.y);
//    quat_tf.setZ(pose.orientation.z);
//    tf::Quaternion q_inv_tf = quat_tf.inverse();
//
//    SX q_inv_fk = SX::horzcat({q_inv_tf.getW(), q_inv_tf.getX(), q_inv_tf.getY(), q_inv_tf.getZ()});
//
//    SX rot = q_fk *q_inv_fk;
//    SX rot_xyz = SX::horzcat({rot(1), rot(2), rot(3)});

//    // Calculate squared distance
    SX dist = dot(p,p);

    SX barrier;
    barrier = exp((min_dist - sqrt(dist))/0.01);

    // Desired endeffector pose
    KDL::Frame frame;
    tf::poseMsgToKDL(pose, frame);

    double roll, pitch, yaw;
    frame.M.GetRPY(roll, pitch, yaw);
    ROS_WARN_STREAM("rpy: " << roll << ", " << pitch << ", " << yaw);

    SX x_desired = SX::horzcat({pose.position.x, pose.position.y, pose.position.z});


    SX x_rot_desired = SX::horzcat({roll, pitch, yaw});

    SX R = 10 * SX::vertcat({1, 1, 1, 1, 1, 1, 1});

    SX energy = dot(dot(u_,sqrt(R)), dot(u_,sqrt(R)));
    // Objective function
    SX L = dot(p - x_desired,p - x_desired)  + dot(x_rot - x_rot_desired, x_rot - x_rot_desired); // + barrier;

    // Create Euler integrator function
    Function F = create_integrator(state_dim_, control_dim_, time_horizon_, num_shooting_nodes_, qdot, x_, u_, L);

    // Total number of NLP variables
    int NV = state_dim_*(num_shooting_nodes_+1) + control_dim_*num_shooting_nodes_;

    // Declare variable vector for the NLP
    MX V = MX::sym("V",NV);

    // NLP variable bounds and initial guess
    vector<double> v_min,v_max,v_init;

    // Offset in V
    int offset=0;

    // State at each shooting node and control for each shooting interval
    vector<MX> X, U;
    for(unsigned int k=0; k<num_shooting_nodes_; ++k)
    {
        // Local state
        X.push_back( V.nz(Slice(offset,offset+state_dim_)));

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
        offset += state_dim_;

        // Local control
        U.push_back( V.nz(Slice(offset,offset+control_dim_)));
        v_min.insert(v_min.end(), u_min.begin(), u_min.end());
        v_max.insert(v_max.end(), u_max.begin(), u_max.end());
        v_init.insert(v_init.end(), u_init_.begin(), u_init_.end());
        offset += control_dim_;
    }

    // State at end
    X.push_back(V.nz(Slice(offset,offset+state_dim_)));
    v_min.insert(v_min.end(), xf_min.begin(), xf_min.end());
    v_max.insert(v_max.end(), xf_max.begin(), xf_max.end());
    v_init.insert(v_init.end(), x_init.begin(), x_init.end());
    offset += state_dim_;

    // Make sure that the size of the variable vector is consistent with the number of variables that we have referenced
    casadi_assert(offset==NV);

    // Objective function
    MX J = 0;

    //Constraint function and bounds
    vector<MX> g;

    // Loop over shooting nodes
    for(unsigned int k=0; k<num_shooting_nodes_; ++k)
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
    Eigen::VectorXd q_dot = Eigen::VectorXd::Zero(state_dim_);
    u_init_.clear();

    for(int i=0; i<1; ++i)  // Copy only the first optimal control sequence
    {
        for(int j=0; j<control_dim_; ++j)
        {
            q_dot[j] = V_opt.at(state_dim_ + j);
        }
    }

    // Safe optimal control sequence at time t_k and take it as inital guess at t_k+1
    for(int i=0; i < control_dim_; ++i)
    {
        u_init_.push_back( q_dot(i) );
    }
//    u_init_.push_back(0);   // Not optimized

    return q_dot;
}

Function CobNonlinearMPC::create_integrator(const unsigned int state_dim, const unsigned int control_dim, const double T,
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
