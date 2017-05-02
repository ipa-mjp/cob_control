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
 *   ROS stack name: cob_driver
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
 ****************************************************************/

#ifndef COB_NONLINEAR_MPC_COB_NONLINEAR_MPC_H
#define COB_NONLINEAR_MPC_COB_NONLINEAR_MPC_H

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>

#include <urdf/model.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/frames.hpp>

#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <ctime>
#include <casadi/casadi.hpp>

using namespace casadi;
using namespace std;


// Only for revolute joints!
struct DH
{
    double alpha;
    double a;
    double d;
    std::string theta;
};


class CobNonlinearMPC
{
private:
    ros::NodeHandle nh_;
    std::vector<std::string> transformation_names_;
    DH dh_param;
    std::vector<DH> dh_params;
    std::vector<SX> transformation_vector;

    int state_dim_;
    int control_dim_;
    int num_shooting_nodes_;
    double time_horizon_;

    // Declare variables
    SX u_;
    SX x_;
    SX fk_;
    SX fk_link4_;
    SX p_;
    tf::TransformListener tf_listener_;

    // Constraints
    vector<double> state_path_constraints_min_, state_path_constraints_max_;
    vector<double> state_terminal_constraints_min_, state_terminal_constraints_max_;
    vector<double> input_constraints_min_, input_constraints_max_;

    ros::Subscriber jointstate_sub_;
    ros::Subscriber odometry_sub_;
    ros::Subscriber pose_sub_;

    ros::Publisher base_vel_pub_;
    ros::Publisher pub_;

    KDL::JntArray joint_state_;
    KDL::JntArray odometry_state_;
    vector<double> u_init_;


public:
    CobNonlinearMPC()
    {
//            u_init_ = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
        u_init_ = { 0, 0, 0, 0, 0, 0, 0 };
    }
    ~CobNonlinearMPC(){}


    bool initialize();

    void jointstateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);

    void poseCallback(const geometry_msgs::Pose::ConstPtr& msg);
    KDL::JntArray getJointState();

    Eigen::MatrixXd mpc_step(const geometry_msgs::Pose pose,
                             const KDL::JntArray& state);

    Function create_integrator(const unsigned int state_dim, const unsigned int control_dim, const double T,
                               const unsigned int N, SX ode, SX x, SX u, SX L);


};


#endif  // COB_NONLINEAR_MPC_COB_NONLINEAR_MPC_H
