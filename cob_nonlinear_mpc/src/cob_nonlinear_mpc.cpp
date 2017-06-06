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


    // nh_nmpc
    if (!nh_nmpc.getParam("transformations", transformation_names_))
    {
        ROS_ERROR("Parameter 'transformation_names' not set");
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
        ROS_ERROR("Parameter 'state_dim' not set");
        return false;
    }
    if (!nh_nmpc.getParam("control_dim", control_dim_))
    {
        ROS_ERROR("Parameter 'control_dim' not set");
        return false;
    }
    if (!nh_nmpc.getParam("base/base_active", base_active_))
    {
        ROS_ERROR("Parameter 'base/base_active' not set");
        return false;
    }
    if (!nh_nmpc.getParam("base/transformations", transformation_names_base_))
    {
        ROS_ERROR("Parameter 'base/transformations' not set");
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

    if(base_active_)
    {
        for(unsigned int i = 0; i < transformation_names_base_.size(); i++)
        {
            DH param;

            if (!nh_nmpc_base_dh.getParam(transformation_names_base_.at(i)+"/type", param.type))
            {
                ROS_ERROR("Parameter 'type' not set");
                return false;
            }

            if (!nh_nmpc_base_dh.getParam(transformation_names_base_.at(i)+"/theta", param.theta))
            {
                ROS_ERROR("Parameter 'theta' not set");
                return false;
            }
            if (!nh_nmpc_base_dh.getParam(transformation_names_base_.at(i)+"/d", param.d))
            {
                ROS_ERROR("Parameter 'd' not set");
                return false;
            }

            if (!nh_nmpc_base_dh.getParam(transformation_names_base_.at(i)+"/a", param.a))
            {
                ROS_ERROR("Parameter 'a' not set");
                return false;
            }

            if (!nh_nmpc_base_dh.getParam(transformation_names_base_.at(i)+"/alpha", param.alpha))
            {
                ROS_ERROR("Parameter 'alpha' not set");
                return false;
            }

            dh_params_base_.push_back(param);
        }
    }


    // nh_nmpc_dh
    for(unsigned int i = 0; i < transformation_names_.size(); i++)
    {
        DH param;

        if (!nh_nmpc_dh.getParam(transformation_names_.at(i)+"/type", param.type))
        {
            ROS_ERROR("Parameter 'type' not set");
            return false;
        }

        if (!nh_nmpc_dh.getParam(transformation_names_.at(i)+"/theta", param.theta))
        {
            ROS_ERROR("Parameter 'theta' not set");
            return false;
        }
        if (!nh_nmpc_dh.getParam(transformation_names_.at(i)+"/d", param.d))
        {
            ROS_ERROR("Parameter 'd' not set");
            return false;
        }

        if (!nh_nmpc_dh.getParam(transformation_names_.at(i)+"/a", param.a))
        {
            ROS_ERROR("Parameter 'a' not set");
            return false;
        }

        if (!nh_nmpc_dh.getParam(transformation_names_.at(i)+"/alpha", param.alpha))
        {
            ROS_ERROR("Parameter 'alpha' not set");
            return false;
        }

        dh_params.push_back(param);
    }

    // Casadi symbolics
    u_ = SX::sym("u", state_dim_);  // control
    x_ = SX::sym("x", control_dim_); // states

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

    /// parse robot_description and generate KDL chains
    KDL::Tree my_tree;
    if (!kdl_parser::treeFromParam("/robot_description", my_tree))
    {
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }

    my_tree.getChain(chain_base_link_, chain_tip_link_, chain_);
    if (chain_.getNrOfJoints() == 0)
    {
        ROS_ERROR("Failed to initialize kinematic chain");
        return false;
    }
    ROS_INFO_STREAM("Number of joints:" << chain_.getNrOfJoints());
    ROS_INFO_STREAM("Number of segments:" << chain_.getNrOfSegments());
    dof = joint_names.size();

    /// parse robot_description and set velocity limits
    urdf::Model model;
    if (!model.initParam("/robot_description"))
    {
        ROS_ERROR("Failed to parse urdf file for JointLimits");
        return false;
    }
    ROS_WARN("Robot Description loaded...");
    std::vector<double> joint_params_;
    urdf::Vector3 position;

    std::vector<KDL::Joint> joints;
    KDL::Frame F;
    double roll,pitch,yaw;
    for (uint16_t i = 0; i < chain_.getNrOfSegments(); i++)
    {
        joints.push_back(chain_.getSegment(i).getJoint());
        ROS_INFO_STREAM("Chain segment "<< chain_.getSegment(i).getName());
        F=chain_.getSegment(i).getFrameToTip();
        F.M.GetRPY(roll,pitch,yaw);
        ROS_INFO_STREAM("Chain frame "<< " X: " << F.p.x()<< " Y: " << F.p.y()<< " Z: "<<F.p.z());
        ROS_INFO_STREAM("Chain frame "<< " ROLL: " << roll<< " PITCH: " << pitch<< " YAW: "<<yaw);


    }

    // JointNames
    std::vector<KDL::Vector> joint_origins;
    for (uint16_t i = 0; i < joints.size(); i++)
    {
        joint_origins.push_back(joints[i].JointOrigin());
        ROS_INFO_STREAM("Joint name "<< joints[i].getName()<< " type: " <<joints[i].getType() << " origin: " << joint_origins[i].x());
        ROS_INFO_STREAM("Joint origin "<< " X: " << joint_origins[i].x()<< " Y: " << joint_origins[i].y()<< " Z: " << joint_origins[i].z());

    }

    KDL::Vector pos;
    KDL::Rotation rot;
    std::vector<KDL::Frame> joint_frames;
    std::vector<KDL::Frame> F_previous;
    for (uint16_t i = 0; i < chain_.getNrOfSegments(); i++)
    {
        if(joints[i].getType()==8)
        {
            ROS_INFO("Fixed joint");
            ROS_INFO_STREAM("Chain segment "<< chain_.getSegment(i).getName());
            if(i==0)
                F_previous.push_back(chain_.getSegment(i).getFrameToTip());
            else
                F_previous.push_back(F_previous.at(i-1)*chain_.getSegment(i).getFrameToTip());

            ROS_INFO_STREAM("Joint position "<< " X: " << F_previous.at(i).p.x()<< " Y: " << F_previous.at(i).p.y()<< " Z: " << F_previous.at(i).p.z());
            rot=F_previous.at(i).M;
            ROS_WARN("Rotation matrix %f %f %f \n %f %f %f \n %f %f %f \n",rot(0,0),rot(0,1),rot(0,2),rot(1,0),rot(1,1),rot(1,2),rot(2,0),rot(2,1),rot(2,2));
            ROS_INFO_STREAM("Joint position of transformation"<< " X: " << F_previous.at(i).p.x()<< " Y: " << F_previous.at(i).p.y()<< " Z: " << F_previous.at(i).p.z());
        }
        if(joints[i].getType()==0){
            ROS_INFO("Rotational joint");
            ROS_INFO_STREAM("Joint name "<< chain_.getSegment(i).getJoint().getName());
            F_previous.push_back(F_previous.at(i-1)*chain_.getSegment(i).getFrameToTip());
            pos=F_previous.at(i).p;
            if(joint_frames.size()==0){
                ROS_INFO("FIRST JOINT");
                joint_frames.push_back(F_previous.at(i));
                rot=F_previous.at(i).M;
                pos=F_previous.at(i).p;
            }
            else{
                joint_frames.push_back(chain_.getSegment(i).getFrameToTip());
                rot=chain_.getSegment(i).getFrameToTip().M;
                pos=chain_.getSegment(i).getFrameToTip().p;
            }
            ROS_INFO_STREAM("Joint position "<< " X: " << pos.x()<< " Y: " << pos.y()<< " Z: " << pos.z());
            ROS_WARN("Rotation matrix %f %f %f \n %f %f %f \n %f %f %f \n",rot(0,0),rot(0,1),rot(0,2),rot(1,0),rot(1,1),rot(1,2),rot(2,0),rot(2,1),rot(2,2));
            ROS_INFO_STREAM("Joint position of transformation"<< " X: " << pos.x()<< " Y: " << pos.y()<< " Z: " << pos.z());            //F_previous.p= pos;
        }
    }

    SX T = SX::sym("T",4,4);

    //Base config
    T(0,0) = cos(x_(2)); T(0,1) = -sin(x_(2));  T(0,2) = 0.0; T(0,3) = x_(0);
    T(1,0) = sin(x_(2)); T(1,1) = cos(x_(2));   T(1,2) = 0.0; T(1,3) = x_(1);
    T(2,0) = 0.0;        T(2,1) = 0.0;          T(2,2) = 1.0; T(2,3) = 0;
    T(3,0) = 0.0;        T(3,1) = 0.0;          T(3,2) = 0.0; T(3,3) = 1.0;

    fk_base_ = T;

    for(int i=0;i<joint_frames.size();i++){

        rot=joint_frames.at(i).M;
        pos=joint_frames.at(i).p;
        ROS_WARN("Rotation matrix %f %f %f \n %f %f %f \n %f %f %f \n",joint_frames.at(i)(0,0),joint_frames.at(i)(0,1),joint_frames.at(i)(0,2),joint_frames.at(i)(1,0),joint_frames.at(i)(1,1),joint_frames.at(i)(1,2),joint_frames.at(i)(2,0),joint_frames.at(i)(2,1),joint_frames.at(i)(2,2));
        ROS_INFO_STREAM("Joint position of transformation"<< " X: " << pos.x()<< " Y: " << pos.y()<< " Z: " << pos.z());
        T(0,0) = 1.0*rot(0,0)*cos(x_(i+3))+rot(0,1)*sin(x_(i+3));
        T(0,1) = -1.0*rot(0,0)*sin(x_(i+3))+rot(0,1)*cos(x_(i+3));
        T(0,2) = 1.0*rot(0,2); T(0,3) = 1.0*pos.x();
        T(1,0) = 1.0*rot(1,0)*cos(x_(i+3))+rot(1,1)*sin(x_(i+3));
        T(1,1) = -1.0*rot(1,0)*sin(x_(i+3))+rot(1,1)*cos(x_(i+3));
        T(1,2) = 1.0*rot(1,2); T(1,3) = 1.0*pos.y();
        T(2,0) = 1.0*rot(2,0)*cos(x_(i+3))+rot(2,1)*sin(x_(i+3));
        T(2,1) = -1.0*rot(2,0)*sin(x_(i+3))+rot(2,1)*cos(x_(i+3));
        T(2,2) = 1.0*rot(2,2); T(2,3) = 1.0*pos.z();
        T(3,0) = 0.0; T(3,1) = 0.0; T(3,2) = 0.0; T(3,3) = 1.0;

        T_BVH p;
        p.T = T;

        transform_vec_bvh_.push_back(p);
    }

    // Get Endeffector FK
    for(int i=0; i< transform_vec_bvh_.size(); i++)
    {
        if(base_active_)
        {
            if(i==0)
            {   ROS_WARN("BASE IS ACTIVE");
                fk_ = mtimes(fk_base_,transform_vec_bvh_.at(i).T);
            }
            else
            {
                fk_ = mtimes(fk_,transform_vec_bvh_.at(i).T);
            }
        }
        else
        {
            if(i==0)
            {
                fk_ = transform_vec_bvh_.at(i).T;
            }
            else
            {
                fk_ = mtimes(fk_,transform_vec_bvh_.at(i).T);
            }
        }
        fk_vector_.push_back(fk_); // stacks up multiplied transformation until link n
    }


    for(int i = 0; i < control_dim_; i++)
    {
        u_init_.push_back(0);
    }

    joint_state_ = KDL::JntArray(7);
    odometry_state_ = KDL::JntArray(3);
    jointstate_sub_ = nh_.subscribe("joint_states", 1, &CobNonlinearMPC::jointstateCallback, this);
    odometry_sub_ = nh_.subscribe("base/odometry", 1, &CobNonlinearMPC::odometryCallback, this);
    pose_sub_ = nh_.subscribe(nh_.getNamespace()+"/command_pose", 1, &CobNonlinearMPC::poseCallback, this);

    base_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("base/command", 1);
    pub_ = nh_.advertise<std_msgs::Float64MultiArray>(nh_.getNamespace()+"/joint_group_velocity_controller/command", 1);

    ROS_WARN_STREAM(nh_.getNamespace() << "/NMPC...initialized!");
    return true;
}

void CobNonlinearMPC::poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    KDL::JntArray state = getJointState();

    Eigen::MatrixXd qdot = mpc_step(*msg, state);

    geometry_msgs::Twist base_vel_msg;
    std_msgs::Float64MultiArray vel_msg;

    base_vel_msg.linear.x = qdot(0);
    base_vel_msg.linear.y = qdot(1);
    base_vel_msg.linear.z = 0;
    base_vel_msg.angular.x = 0;
    base_vel_msg.angular.y = 0;
    base_vel_msg.angular.z = qdot(2);

    base_vel_pub_.publish(base_vel_msg);


    for (unsigned int i = 3; i < 10; i++)
    {
        vel_msg.data.push_back(qdot(i));
    }
    pub_.publish(vel_msg);

//    for (unsigned int i = 0; i < 7; i++)
//    {
//        vel_msg.data.push_back(static_cast<double>(qdot(i)));
//    }
//    pub_.publish(vel_msg);
}


void CobNonlinearMPC::jointstateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{

    KDL::JntArray q_temp = joint_state_;

    joint_names = {"arm_left_1_joint", "arm_left_2_joint", "arm_left_3_joint", "arm_left_4_joint", "arm_left_5_joint", "arm_left_6_joint", "arm_left_7_joint"};
//    joint_names = {"arm_1_joint","arm_2_joint","arm_3_joint","arm_4_joint","arm_5_joint","arm_6_joint","arm_7_joint"};

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
    double ysqr = msg->pose.pose.orientation.y * msg->pose.pose.orientation.y;
    double t3 = +2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
    double t4 = +1.0 - 2.0 * (ysqr + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);

    temp(2) = std::atan2(t3, t4);

    odometry_state_ = temp;
}


KDL::JntArray CobNonlinearMPC::getJointState()
{
    KDL:: JntArray tmp(joint_state_.rows() + odometry_state_.rows());
//    KDL:: JntArray tmp(joint_state_.rows());

//    tmp = this->odometry_state_;

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
    double min_dist = 0.4;

    // Bounds and initial guess for the control
    vector<double> u_min =  input_constraints_min_;
    vector<double> u_max  = input_constraints_max_;

    // Bounds and initial guess for the state
    vector<double> x0_min;
    vector<double> x0_max;
    vector<double> x_init;
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

    // ODE right hand side and quadrature
    SX qdot = SX::vertcat({u_});

    // Current Quaternion and Position Vector.
    double kappa = 0.001; // Small regulation term for numerical stability for the NLP

    SX q_c = SX::vertcat({
        0.5 * sqrt(fk_(0,0) + fk_(1,1) + fk_(2,2) + 1.0 + kappa),
        0.5 * (sign((fk_(2,1) - fk_(1,2)))) * sqrt(fk_(0,0) - fk_(1,1) - fk_(2,2) + 1.0 + kappa),
        0.5 * (sign((fk_(0,2) - fk_(2,0)))) * sqrt(fk_(1,1) - fk_(2,2) - fk_(0,0) + 1.0 + kappa),
        0.5 * (sign((fk_(1,0) - fk_(0,1)))) * sqrt(fk_(2,2) - fk_(0,0) - fk_(1,1) + 1.0 + kappa)
    });

    SX p_c = SX::vertcat({fk_(0,3), fk_(1,3), fk_(2,3)});

    // Desired Goal-pose
    SX x_d = SX::vertcat({pose.position.x, pose.position.y, pose.position.z});
    SX q_d = SX::vertcat({pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z});

    // Base constraint
    SX bvh_p = SX::vertcat({fk_base_(0,3), fk_base_(1,3), fk_base_(2,3)+0.1});
    SX bvh_p2 = SX::vertcat({fk_base_(0,3), fk_base_(1,3), fk_base_(2,3)+0.5});
    SX bvh_p3 = SX::vertcat({fk_base_(0,3), fk_base_(1,3), fk_base_(2,3)+0.7});
    SX bvh_p4 = SX::vertcat({fk_base_(0,3), fk_base_(1,3), fk_base_(2,3)+0.9});
    SX bvh_p5 = SX::vertcat({fk_base_(0,3), fk_base_(1,3), fk_base_(2,3)+1.1});
    SX bvh_p6 = SX::vertcat({fk_base_(0,3), fk_base_(1,3), fk_base_(2,3)+1.4});

    // Prevent collision with Base_link
    SX barrier;
    SX dist;
//    SX dist = dot(p_c,p_c);
//    barrier = exp((min_dist - sqrt(dist))/0.01);
//
//    SX BVH_barrier;
//


    for(int i = 0; i < fk_vector_.size(); i++)
    {
        if(i>0)
        {
            SX tmp = fk_vector_.at(i);
            SX p = SX::vertcat({tmp(0,3),tmp(1,3),tmp(2,3)});

            if(i==1)
            {
                dist = dot(bvh_p - p,bvh_p - p);
                barrier = exp((min_dist - sqrt(dist))/0.01);

                dist = dot(bvh_p2 - p,bvh_p2 - p);
                barrier += exp((0.25 - sqrt(dist))/0.01);

                dist = dot(bvh_p3 - p,bvh_p3 - p);
                barrier += exp((0.2 - sqrt(dist))/0.01);

                dist = dot(bvh_p4 - p,bvh_p4 - p);
                barrier += exp((0.2 - sqrt(dist))/0.01);

                dist = dot(bvh_p5 - p,bvh_p5 - p);
                barrier += exp((0.2 - sqrt(dist))/0.01);

                dist = dot(bvh_p6 - p,bvh_p6 - p);
                barrier += exp((0.25 - sqrt(dist))/0.01);
            }
            else
            {
                dist = dot(bvh_p - p,bvh_p - p);
                barrier += exp((min_dist - sqrt(dist))/0.01);

                dist = dot(bvh_p2 - p,bvh_p2 - p);
                barrier += exp((0.25 - sqrt(dist))/0.01);

                dist = dot(bvh_p3 - p,bvh_p3 - p);
                barrier += exp((0.2 - sqrt(dist))/0.01);

                dist = dot(bvh_p4 - p,bvh_p4 - p);
                barrier += exp((0.2 - sqrt(dist))/0.01);

                dist = dot(bvh_p5 - p,bvh_p5 - p);
                barrier += exp((0.2 - sqrt(dist))/0.01);

                dist = dot(bvh_p6 - p,bvh_p6 - p);
                barrier += exp((0.25 - sqrt(dist))/0.01);
            }
        }
    }
    SX q_c_inverse = SX::vertcat({q_c(0), -q_c(1), -q_c(2), -q_c(3)});
//    q_c_inverse = q_c_inverse / sqrt(dot(q_c_inverse,q_c_inverse));

    SX e_quat= quaternion_product(q_c_inverse,q_d);

//    q_c_inverse = q_c_inverse / sqrt(dot(q_c_inverse,q_c_inverse));

    SX error_attitute = SX::vertcat({ e_quat(1), e_quat(2), e_quat(3)});
    SX R = 1*SX::vertcat({10, 10, 10, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1});
    SX energy = dot(sqrt(R)*u_,sqrt(R)*u_);

//    SX L = 10 * dot(p_c-x_d,p_c-x_d) + 10 * dot(q_c - q_d, q_c - q_d) + energy + barrier;
    SX L = 10*dot(p_c-x_d,p_c-x_d) + energy + 10 * dot(error_attitute,error_attitute) + barrier;

    // Create Euler integrator function
    Function F = create_integrator(state_dim_, control_dim_, time_horizon_, num_shooting_nodes_, qdot, x_, u_, L);

//    // Generate C-code
//    F.generate("nmpc");
//
//    Importer("nmpc.c","clang");
//
//    // Compile the C-code to a shared library
//    string compile_command = "gcc -fPIC -shared -O3 nmpc.c -o nmpc.so";
//    int flag = system(compile_command.c_str());
//    casadi_assert_message(flag==0, "Compilation failed");
//
////
//    // Use CasADi's "external" to load the compiled function
//    F = external("nmpc");
//
//    F = external("F", "./nmpc.so");

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
    opts["ipopt.max_iter"] = 20;
//    opts["ipopt.hessian_approximation"] = "limited-memory";
//    opts["ipopt.hessian_constant"] = "yes";
    opts["ipopt.linear_solver"] = "ma27";
    opts["ipopt.print_level"] = 0;
    opts["print_time"] = true;
    opts["expand"] = true;  // Removes overhead, not sure if this command does the same as in the create_integrator function !

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
//    Eigen::VectorXd x_new = Eigen::VectorXd::Zero(state_dim_);
    vector<double> x_new;
    SX sx_x_new;
    u_init_.clear();
    x_new.clear();

    for(int i=0; i<1; ++i)  // Copy only the first optimal control sequence
    {
        for(int j=0; j<control_dim_; ++j)
        {
            q_dot[j] = V_opt.at(state_dim_ + j);
            x_new.push_back(V_opt.at(j));
        }
    }

    sx_x_new = SX::vertcat({x_new});
    // Safe optimal control sequence at time t_k and take it as inital guess at t_k+1
    for(int i=0; i < control_dim_; ++i)
    {
        u_init_.push_back(q_dot(i));
//        u_init_.push_back(V_opt.at(state_dim_ + control_dim_ + i));
    }

    geometry_msgs::Point point;
    point.x = 0;
    point.y = 0;
    point.z = 0;

    Function fk_sx = Function("fk_sx", {x_}, {bvh_p});
    Function fk_sx2 = Function("fk_sx2", {x_}, {bvh_p2});
    Function fk_sx3 = Function("fk_sx3", {x_}, {bvh_p3});
    Function fk_sx4 = Function("fk_sx4", {x_}, {bvh_p4});
    Function fk_sx5 = Function("fk_sx3", {x_}, {bvh_p5});
    Function fk_sx6 = Function("fk_sx4", {x_}, {bvh_p6});

    SX result = fk_sx(sx_x_new).at(0);
    point.x = (double)result(0);
    point.y = (double)result(1);
    point.z = (double)result(2);

    visualizeBVH(point, 0.4, 1);

    result = fk_sx2(sx_x_new).at(0);
    point.x = (double)result(0);
    point.y = (double)result(1);
    point.z = (double)result(2);

    visualizeBVH(point, 0.25, 2);

    result = fk_sx3(sx_x_new).at(0);
    point.x = (double)result(0);
    point.y = (double)result(1);
    point.z = (double)result(2);

    visualizeBVH(point, 0.2, 3);

    result = fk_sx4(sx_x_new).at(0);
    point.x = (double)result(0);
    point.y = (double)result(1);
    point.z = (double)result(2);

    visualizeBVH(point, 0.2, 4);

    result = fk_sx5(sx_x_new).at(0);
    point.x = (double)result(0);
    point.y = (double)result(1);
    point.z = (double)result(2);

    visualizeBVH(point, 0.2, 5);

    result = fk_sx6(sx_x_new).at(0);
    point.x = (double)result(0);
    point.y = (double)result(1);
    point.z = (double)result(2);

    visualizeBVH(point, 0.25, 6);

//    for(int i = 0; i < bvh_points_.size(); i++)
//    {
//        Function fk_sx = Function("fk_sx", {x_}, {bvh_points_.at(i)});
//
//        SX result = fk_sx(sx_x_new).at(0);
//        point.x = (double)result(0);
//        point.y = (double)result(1);
//        point.z = (double)result(2);
//
//        visualizeBVH(point, min_dist, i);
//    }

    Function fk_test = Function("fk_", {x_}, {p_c});

        vector<double> state_vec;
        for(int i = 0; i < state.rows(); i++)
        {
            state_vec.push_back((double)state.data(i));
        }
        sx_x_new = SX::vertcat({state_vec});
        SX state_v = SX::vertcat({state_vec});

        SX test_v = fk_test(sx_x_new).at(0);

        state_vec.clear();
        state_vec.push_back((double)test_v(0));
        state_vec.push_back((double)test_v(1));
        state_vec.push_back((double)test_v(2));
        ROS_INFO_STREAM("Joint values:" <<x_new);
        ROS_WARN_STREAM("Goal: \n" << pose.position.x <<", " << pose.position.y << ", " << pose.position.z);
        ROS_WARN_STREAM("Current Position: \n" << state_vec);

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
