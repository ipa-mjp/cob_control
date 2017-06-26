/*
 * forward_kinematics.h
 *
 *  Created on: 18.06.2017
 *      Author: chris
 */

#ifndef FORWARD_KINEMATICS_H
#define FORWARD_KINEMATICS_H

#include <ros/ros.h>
#include <casadi/casadi.hpp>

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

#include <cob_nonlinear_mpc/data_types.h>
#include <cob_nonlinear_mpc/robot.h>


using namespace casadi;
using namespace std;


class ForwardKinematics
{
    private:
        ros::NodeHandle nh_;
        std::vector<SX> fk_vector_;
        SX T_base_;
        std::vector<T_BVH> transformation_vector_; // Vector of symbolic transformation matrices from n-1 to n
        std::vector<T_BVH> jacobian_vector_; // Vector of symbolic transformation matrices from n-1 to n
        SX u_;
        SX x_;

    public:
        ForwardKinematics(){}
        ~ForwardKinematics(){}

        void symbolic_fk(Robot &robot);
        void symbolic_jacobian(Robot &robot);
        std::vector<SX> getFkVector();
        SX getFkBase();
        std::vector<T_BVH> getTransformVector();

        void setX(SX x);
        void setU(SX u);
        SX getX();
        SX getU();

};
#endif
