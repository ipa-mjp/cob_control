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

#ifndef BOUNDING_VOLUME_H
#define BOUNDING_VOLUME_H

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

#include <ctime>
#include <casadi/casadi.hpp>

#include <cob_nonlinear_mpc/data_types.h>
#include <cob_nonlinear_mpc/robot.h>
#include <cob_nonlinear_mpc/forward_kinematics.h>


using namespace casadi;
using namespace std;

class BoundingVolume
{
private:

    ros::NodeHandle nh_;
    visualization_msgs::MarkerArray marker_array_;
    std::map<string,vector<vector<SX>>> bv_mat;

    vector<double> bvb_positions_;
    vector<double> bvb_radius_;

    ros::Publisher marker_pub_;
    Robot robot_;
    ForwardKinematics fk_;

public:
    BoundingVolume()
    {
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("nmpc/bvh", 1);
    }
    ~BoundingVolume(){}
    void plotBoundingVolumes(SX x_current);
    void addCollisionBall(const geometry_msgs::Point point, double radius, int id);

    SX getOutputConstraints();
    void generate_bounding_volumes();

    bool setBVBpositions(vector<double> bvb_pos);
    bool setBVBradius(vector<double> bvb_rad);
    void setRobot(Robot robot);
    Robot getRobot();
    void setForwardKinematic(ForwardKinematics fk);
    ForwardKinematics getForwardKinematic();
};

#endif  // BOUNDING_VOLUME_H
