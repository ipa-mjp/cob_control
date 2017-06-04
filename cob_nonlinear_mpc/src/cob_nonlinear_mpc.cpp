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

    // Casadi symbolics
    u_ = SX::sym("u", control_dim_);  // control
    x_ = SX::sym("x", state_dim_); // states

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

    SX T = SX::sym("T",4,4);
	if(base_active_){
        //Base config
        T(0,0) = cos(x_(2)); T(0,1) = -sin(x_(2));  T(0,2) = 0.0; T(0,3) = x_(0);
        T(1,0) = sin(x_(2)); T(1,1) = cos(x_(2));   T(1,2) = 0.0; T(1,3) = x_(1);
        T(2,0) = 0.0;        T(2,1) = 0.0;          T(2,2) = 1.0; T(2,3) = 0;
        T(3,0) = 0.0;        T(3,1) = 0.0;          T(3,2) = 0.0; T(3,3) = 1.0;

        fk_base_ = T;

    }
    int offset;

    for(int i=0;i<robot_.kinematic_chain.getNrOfSegments();i++){

        KDL::Vector pos;
        KDL::Rotation rot;
        rot=robot_.kinematic_chain.getSegment(i).getFrameToTip().M;
        pos=robot_.kinematic_chain.getSegment(i).getFrameToTip().p;
#ifdef __DEBUG__
        ROS_WARN("Rotation matrix %f %f %f \n %f %f %f \n %f %f %f \n",rot(0,0),rot(0,1),rot(0,2),rot(1,0),rot(1,1),rot(1,2),rot(2,0),rot(2,1),rot(2,2));
        ROS_INFO_STREAM("Joint position of transformation"<< " X: " << pos.x()<< " Y: " << pos.y()<< " Z: " << pos.z());
#endif
        if(base_active_){
            T(0,0) = rot(0,0)*cos(x_(i+3))+rot(0,1)*sin(x_(i+3));
            T(0,1) = -rot(0,0)*sin(x_(i+3))+rot(0,1)*cos(x_(i+3));
            T(0,2) = rot(0,2); T(0,3) = pos.x();
            T(1,0) = rot(1,0)*cos(x_(i+3))+rot(1,1)*sin(x_(i+3));
            T(1,1) = -rot(1,0)*sin(x_(i+3))+rot(1,1)*cos(x_(i+3));
            T(1,2) = rot(1,2); T(1,3) = pos.y();
            T(2,0) = rot(2,0)*cos(x_(i+3))+rot(2,1)*sin(x_(i+3));
            T(2,1) = -rot(2,0)*sin(x_(i+3))+rot(2,1)*cos(x_(i+3));
            T(2,2) = rot(2,2); T(2,3) = pos.z();
            T(3,0) = 0.0; T(3,1) = 0.0; T(3,2) = 0.0; T(3,3) = 1.0;
        }
        else{
            T(0,0) = rot(0,0)*cos(x_(i))+rot(0,1)*sin(x_(i));
            T(0,1) = -rot(0,0)*sin(x_(i))+rot(0,1)*cos(x_(i));
            T(0,2) = rot(0,2); T(0,3) = pos.x();
            T(1,0) = rot(1,0)*cos(x_(i))+rot(1,1)*sin(x_(i));
            T(1,1) = -rot(1,0)*sin(x_(i))+rot(1,1)*cos(x_(i));
            T(1,2) = rot(1,2); T(1,3) = pos.y();
            T(2,0) = rot(2,0)*cos(x_(i))+rot(2,1)*sin(x_(i));
            T(2,1) = -rot(2,0)*sin(x_(i))+rot(2,1)*cos(x_(i));
            T(2,2) = rot(2,2); T(2,3) = pos.z();
            T(3,0) = 0.0; T(3,1) = 0.0; T(3,2) = 0.0; T(3,3) = 1.0;
        }

        T_BVH p;
        p.link = robot_.kinematic_chain.getSegment(i).getName();
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

    // Get bounding volume forward kinematics
    for(int i=0; i<transform_vec_bvh_.size(); i++)
    {
        T_BVH bvh = transform_vec_bvh_.at(i);
        std::vector<SX> bvh_arm;
        if(i-1<0)
        {
            SX transform = mtimes(fk_vector_.at(i),bvh.T);
            SX tmp = SX::vertcat({transform(0,3), transform(1,3), transform(2,3)});
            bvh_arm.push_back(tmp);
            bvh_matrix[bvh.link].push_back(bvh_arm);

            if(bvh.constraint)
            {
                bvh_arm.clear();
                tmp.clear();
                transform = mtimes(fk_vector_.at(i),bvh.BVH_p);
                tmp = SX::vertcat({transform(0,3), transform(1,3), transform(2,3)});
                bvh_arm.push_back(tmp);
                bvh_matrix[bvh.link].push_back(bvh_arm);
            }
        }
        else
        {
            bvh_arm.clear();
            SX transform = mtimes(fk_vector_.at(i-1),bvh.T);
            SX tmp = SX::vertcat({transform(0,3), transform(1,3), transform(2,3)});
            bvh_arm.push_back(tmp);
            bvh_matrix[bvh.link].push_back(bvh_arm);
            bvh_arm.clear();

            if(bvh.constraint)
            {
                tmp.clear();
                transform = mtimes(fk_vector_.at(i-1),bvh.BVH_p);
                tmp = SX::vertcat({transform(0,3), transform(1,3), transform(2,3)});
                bvh_arm.push_back(tmp);
                bvh_matrix[bvh.link].push_back(bvh_arm);
            }
        }
    }
    if(base_active_)
    {
        for(int i=0; i<bvb_positions_.size(); i++)
        {
            std::vector<SX> base_bvh;
            SX tmp = SX::vertcat({fk_base_(0,3), fk_base_(1,3), fk_base_(2,3)+bvb_positions_.at(i)});
            base_bvh.push_back(tmp);
            bvh_matrix["body"].push_back(base_bvh);
        }

    }


    vector<double> tmp;
    for(int k=0; k < num_shooting_nodes_; ++k)
    {
        tmp.clear();
        for(int i=0; i < control_dim_; ++i)
        {
            tmp.push_back(0);
        }
        u_open_loop_.push_back(tmp);
    }

    for(int k=1; k <= num_shooting_nodes_; ++k)
    {
        tmp.clear();

        for(int i=0; i < state_dim_; ++i)
        {
            tmp.push_back(0);
        }

        x_open_loop_.push_back(tmp);
    }

    joint_state_ = KDL::JntArray(chain_.getNrOfJoints());
    jointstate_sub_ = nh_.subscribe("joint_states", 1, &CobNonlinearMPC::jointstateCallback, this);

    if(base_active_){
        odometry_state_ = KDL::JntArray(3);
        odometry_sub_ = nh_.subscribe("base/odometry", 1, &CobNonlinearMPC::odometryCallback, this);
        base_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("base/command", 1);
    }

    frame_tracker_sub_ = nh_.subscribe("command_pose", 1, &CobNonlinearMPC::FrameTrackerCallback, this);
    pub_ = nh_.advertise<std_msgs::Float64MultiArray>("joint_group_velocity_controller/command", 1);

    ROS_WARN_STREAM(nh_.getNamespace() << "/NMPC...initialized!");
    return true;
}

bool CobNonlinearMPC::process_KDL_tree(){

    /// parse robot_description and generate KDL chains

    if (!kdl_parser::treeFromParam("/robot_description", robot_tree_))
    {
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }

    robot_tree_.getChain(chain_base_link_, chain_tip_link_, robot_.kinematic_chain);
    if (chain_.getNrOfJoints() == 0)
    {
        ROS_ERROR("Failed to initialize kinematic chain");
        return false;
    }

#ifdef __DEBUG__
        ROS_INFO_STREAM("Number of joints:" << robot_.kinematic_chain.getNrOfJoints());
        ROS_INFO_STREAM("Number of segments:" << robot_.kinematic_chain.getNrOfSegments());
#endif

    robot_.dof = joint_names.size();

    // parse robot_description and set velocity limits

    if (!robot_.urdf.initParam("/robot_description"))
    {
        ROS_ERROR("Failed to parse urdf file for JointLimits");
        return false;
    }

    ROS_INFO("Robot Description loaded...");
    std::vector<double> joint_params_;
    urdf::Vector3 position;

    KDL::Frame F;
    double roll,pitch,yaw;
    for (uint16_t i = 0; i < chain_.getNrOfSegments(); i++)
    {
        robot_.joints.push_back(chain_.getSegment(i).getJoint());
        ROS_INFO_STREAM("Chain segment "<< chain_.getSegment(i).getName());
        F=chain_.getSegment(i).getFrameToTip();
        F.M.GetRPY(roll,pitch,yaw);
#ifdef __DEBUG__
        ROS_INFO_STREAM("Chain frame "<< " X: " << F.p.x()<< " Y: " << F.p.y()<< " Z: "<<F.p.z());
        ROS_INFO_STREAM("Chain frame "<< " ROLL: " << roll<< " PITCH: " << pitch<< " YAW: "<<yaw);
#endif
    }
#ifdef __DEBUG__
        // JointNames This can be later deleted
        std::vector<KDL::Vector> joint_origins;
        for (uint16_t i = 0; i < robot_.joints.size(); i++)
        {
            joint_origins.push_back(robot_.joints.at(i).JointOrigin());
            ROS_INFO_STREAM("Joint name "<< robot_.joints.at(i).getName()<< " type: " <<robot_.joints.at(i).getType() << " origin: " << robot_.joint_origins[i].x());
            ROS_INFO_STREAM("Joint origin "<< " X: " << robot_.joint_origins[i].x()<< " Y: " << robot_.joint_origins[i].y()<< " Z: " << robot_.joint_origins[i].z());

        }
#endif

        std::vector<KDL::Segment> joint_frames;
        for (uint16_t i = 0; i < robot_.kinematic_chain.getNrOfSegments(); i++)
        {
            if(robot_.joints.at(i).getType()==8)//fixed joint
            {
                if(i==0)//First joint
                {
                    KDL::Segment link(robot_.kinematic_chain.getSegment(i).getName(),KDL::Joint(KDL::Joint::None),robot_.kinematic_chain.getSegment(i).getFrameToTip(),KDL::RigidBodyInertia::Zero());
                    robot_.forward_kinematics.push_back(link);
                }
                else
                {
                    KDL::Frame f = robot_.forward_kinematics.at(i-1).getFrameToTip()*robot_.kinematic_chain.getSegment(i).getFrameToTip();
                    KDL::Segment link(robot_.kinematic_chain.getSegment(i).getName(),KDL::Joint(KDL::Joint::None),f,KDL::RigidBodyInertia::Zero());
                    robot_.forward_kinematics.push_back(link);
                }

#ifdef __DEBUG__
                KDL::Vector pos;
                KDL::Rotation rot;
                rot=robot_.forward_kinematics.at(i).getFrameToTip().M;
                ROS_INFO("Fixed joint");
                ROS_INFO_STREAM("Chain segment "<< robot_.kinematic_chain.getSegment(i).getName());
                ROS_INFO_STREAM("Joint position "<< " X: " << robot_.forward_kinematics.at(i).getFrameToTip().p.x()<< " Y: " << robot_.forward_kinematics.at(i).getFrameToTip().p.y()<< " Z: " << robot_.forward_kinematics.at(i).getFrameToTip().p.z());
                ROS_WARN("Rotation matrix %f %f %f \n %f %f %f \n %f %f %f \n",rot(0,0),rot(0,1),rot(0,2),rot(1,0),rot(1,1),rot(1,2),rot(2,0),rot(2,1),rot(2,2));
                ROS_INFO_STREAM("Joint position of transformation"<< " X: " << robot_.forward_kinematics.at(i).getFrameToTip().p.x()<< " Y: " << robot_.forward_kinematics.at(i).getFrameToTip().p.y()<< " Z: " << robot_.forward_kinematics.at(i).getFrameToTip().p.z());
#endif
            }
            if(robot_.joints.at(i).getType()==0) //revolute joint
            {

                KDL::Frame f = robot_.forward_kinematics.at(i-1).getFrameToTip()*chain_.getSegment(i).getFrameToTip();
                KDL::Segment link(chain_.getSegment(i).getName(),KDL::Joint(KDL::Joint::None),f,KDL::RigidBodyInertia::Zero());
                robot_.forward_kinematics.push_back(link);
                if(i==0)//First joint
                {
                    KDL::Segment link(chain_.getSegment(i).getName(),KDL::Joint(KDL::Joint::None),chain_.getSegment(i).getFrameToTip(),KDL::RigidBodyInertia::Zero());
                    robot_.forward_kinematics.push_back(link);
                }
                else
                {
                    KDL::Frame f = robot_.forward_kinematics.at(i-1).getFrameToTip()*robot_.kinematic_chain.getSegment(i).getFrameToTip();
                    KDL::Segment link(robot_.kinematic_chain.getSegment(i).getName(),KDL::Joint(KDL::Joint::None),f,KDL::RigidBodyInertia::Zero());
                    robot_.forward_kinematics.push_back(link);
                    // joint_frames its the same as chain_.getSegment(i).getFrameToTip() which is not in the robot variable
                    // in this case robot_.kinematic_chain.getSegment(i).getFrameToTip()

                }
#ifdef __DEBUG__
                rot=robot_.kinematic_chain.getSegment(i).getFrameToTip().M;
                pos=robot_.kinematic_chain.getSegment(i).getFrameToTip().p;
                ROS_INFO("Rotational joint");
                ROS_INFO_STREAM("Joint name "<< chain_.getSegment(i).getJoint().getName());
                ROS_INFO_STREAM("Joint position "<< " X: " << pos.x()<< " Y: " << pos.y()<< " Z: " << pos.z());
                ROS_WARN("Rotation matrix %f %f %f \n %f %f %f \n %f %f %f \n",rot(0,0),rot(0,1),rot(0,2),rot(1,0),rot(1,1),rot(1,2),rot(2,0),rot(2,1),rot(2,2));
                ROS_INFO_STREAM("Joint position of transformation"<< " X: " << pos.x()<< " Y: " << pos.y()<< " Z: " << pos.z());            //F_previous.p= pos;
#endif
            }
        }

}

void CobNonlinearMPC::FrameTrackerCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    KDL::JntArray state = getJointState();

    Eigen::MatrixXd qdot = mpc_step(*msg, state);

    geometry_msgs::Twist base_vel_msg;
    std_msgs::Float64MultiArray vel_msg;

    if(base_active_){
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
    vector<double> u_min =  input_constraints_min_;
    vector<double> u_max  = input_constraints_max_;

    ROS_INFO("Bounds and initial guess for the state");
    vector<double> x0_min;
    vector<double> x0_max;
    vector<double> x_init;
    for(unsigned int i=0; i < state.rows();i++)
    {
        x0_min.push_back(state(i));
        x0_max.push_back(state(i));
        x_init.push_back(state(i));
    }

    x_open_loop_.at(0) = x_init;

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
    SX R = 1*SX::vertcat({100, 100, 100, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1});
    SX energy = dot(sqrt(R)*u_,sqrt(R)*u_);

    // L2 norm of the states
    SX S = 0.1*SX::vertcat({0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1});
    SX motion = dot(sqrt(S)*x_,sqrt(S)*x_);

    // Objective
    SX L = 10*dot(p_c-x_d,p_c-x_d) + energy + 10 * dot(error_attitute,error_attitute) + barrier;

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

        vector<double> x_ol;
        x_ol = x_open_loop_.at(k);

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
        v_init.insert(v_init.end(), x_ol.begin(), x_ol.end());
        offset += state_dim_;

        // Local control via shift initialization
        U.push_back( V.nz(Slice(offset,offset+control_dim_)));
        v_min.insert(v_min.end(), u_min.begin(), u_min.end());
        v_max.insert(v_max.end(), u_max.begin(), u_max.end());

        vector<double> u_ol;
        u_ol = u_open_loop_.at(k);

        v_init.insert(v_init.end(), u_ol.begin(), u_ol.end());
        offset += control_dim_;
    }

    vector<double> x_ol;
    x_ol = x_open_loop_.at(num_shooting_nodes_-1);
    // State at end
    X.push_back(V.nz(Slice(offset,offset+state_dim_)));
    v_min.insert(v_min.end(), xf_min.begin(), xf_min.end());
    v_max.insert(v_max.end(), xf_max.begin(), xf_max.end());
    v_init.insert(v_init.end(), x_ol.begin(), x_ol.end());
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

    opts["ipopt.tol"] = 1e-4;
    opts["ipopt.max_iter"] = 20;
//    opts["ipopt.hessian_approximation"] = "limited-memory";
//    opts["ipopt.hessian_constant"] = "yes";
    opts["ipopt.linear_solver"] = "ma27";
    opts["ipopt.print_level"] = 0;
    opts["print_time"] = true;
    opts["expand"] = true;  // Removes overhead

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
    u_open_loop_.clear();
    x_open_loop_.clear();
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
