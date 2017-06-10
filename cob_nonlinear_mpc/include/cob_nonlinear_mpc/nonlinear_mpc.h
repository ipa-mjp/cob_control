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

#ifndef NONLINEAR_MPC_H
#define NONLINEAR_MPC_H

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
#include <visualization_msgs/MarkerArray.h>
#include <cob_nonlinear_mpc/bounding_volumes.h>
#include <cob_nonlinear_mpc/robot.h>

#include <ctime>
#include <casadi/casadi.hpp>

#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>

#include <Eigen/Core>

using namespace casadi;
using namespace std;

class MPC
{
private:

    int num_shooting_nodes_;
    double time_horizon_;
    int state_dim_;
    int control_dim_;
    int NV;

    vector<vector<double>> u_open_loop_;
    vector<vector<double>> x_open_loop_;

    vector<double> u_min;
    vector<double> u_max;

    vector<double> x_min;
    vector<double> x_max;
    vector<double> xf_min;
    vector<double> xf_max;

    vector<double> u_init_;
    // Symbolic variables
    SX u_; //control symbolic input
    SX x_; //robot symbolic state
    SX q_c; //Current orientation quaternion
    SX pos_c ;//Current cartesian position
    SX pos_target; //Target position
    SX q_target; //target quaternion orientation

    SX fk_; //Forward kinematics
    SX fk_base_; //Base Forward kinematics
    std::vector<SX> fk_vector_; // Forward kinematics for each link

    // State at each shooting node and control for each shooting interval
    // Declare variable vector for the NLP
    MX V ;
    vector<MX> X, U;
    // NLP variable bounds and initial guess
    vector<double> min_state,max_state,init_state;

    vector<double> x_new; //new state after computation

    ros::NodeHandle nh_;

    //COORDINATION
    std::vector<double> weiting;

public:
    MPC(int num_shooting_nodes,double time_horizon ,int state_dim,int control_dim): BV("nmpc/bvh")
    {
        num_shooting_nodes_=num_shooting_nodes;
        time_horizon_=time_horizon;
        state_dim_=state_dim;
        control_dim_=control_dim;
    }
    ~MPC(){}

    // PATH CONSTRAINTS
    vector<double> state_path_constraints_min_, state_path_constraints_max_;
    vector<double> state_terminal_constraints_min_, state_terminal_constraints_max_;
    vector<double> input_constraints_min_, input_constraints_max_;

    vector<double> x0_min;
    vector<double> x0_max;
    vector<double> x_init;

    //Bounding volumes
    BoundingVolume BV;
    SX R ;

    void init();

    int get_num_shooting_nodes();

    double get_time_horizon();

    int get_state_dim();

    int get_control_dim();

    void set_path_constraints(vector<double> state_path_constraints_min,vector<double> state_path_constraints_max);

    void set_state_constraints(vector<double> state_terminal_constraints_min,vector<double> state_terminal_constraints_max);

    void set_input_constraints(vector<double> input_constraints_min,vector<double> input_constraints_max);

    void generate_symbolic_forward_kinematics(Robot* robot);

    SX quaternion_product(SX q1, SX q2);

    SX dual_quaternion_product(SX q1, SX q2);

    KDL::Frame forward_kinematics(const KDL::JntArray& state);

    Eigen::MatrixXd mpc_step(const geometry_msgs::Pose pose, const KDL::JntArray& state, Robot* robot);

    Function create_integrator(const unsigned int state_dim, const unsigned int control_dim, const double T,
                               const unsigned int N, SX ode, SX x, SX u, SX L);

    int init_shooting_node();

    void acceleration_coordination(const KDL::JntArray& state);
};

#endif  // NONLINEAR_MPC_H