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

#include <cob_nonlinear_mpc/cob_nonlinear_mpc.h>
#include <cob_nonlinear_mpc/data_types.h>

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

public:
    BoundingVolume()
    {
    }
    ~BoundingVolume(){}


    void visualizeBVH(const geometry_msgs::Point point, double radius, int id);

    SX getOutputConstraints(Robot &robot);
    void generate_bounding_volumes(Robot &robot, ForwardKinematics &fk);

    bool setBVBpositions(vector<double> bvb_pos);
    bool setBVBradius(vector<double> bvb_rad);
};

#endif  // BOUNDING_VOLUME_H
