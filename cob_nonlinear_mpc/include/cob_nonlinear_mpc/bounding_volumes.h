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

#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>

#include <cob_nonlinear_mpc/robot.h>

using namespace casadi;
using namespace std;

struct T_BVH
{
    SX T;
    SX BVH_p;
    string link;
    bool constraint = false;
};

class BoundingVolume
{
private:

    ros::NodeHandle nh_;
    SX T;
    SX BVH_p;
    string link;
    bool constraint = false;

public:
    BoundingVolume(string topic)
    {
        //marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(topic, 1);
    }
    ~BoundingVolume(){}
    //Bounding volumes
    std::vector<SX> bvh_points_;
    std::vector<T_BVH> transform_vec_bvh_;
    std::vector<SX> transform_base_vec_;

    XmlRpc::XmlRpcValue bvb_;

    vector<double> bvb_positions_;
    vector<double> bvb_radius_;

    ros::Publisher marker_pub_;

    visualization_msgs::MarkerArray marker_array_;

    void visualizeBVH(const geometry_msgs::Point point, double radius, int id);

    std::map<string,vector<vector<SX>>> generate_bounding_volumes(Robot &robot, ForwardKinematics &fk);
};

#endif  // BOUNDING_VOLUME_H
