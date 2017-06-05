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
 * \date Date of creation: June, 2017
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

bool CobNonlinearMPC::process_KDL_tree(){

    /// parse robot_description and generate KDL chains

    if (!kdl_parser::treeFromParam("/robot_description", robot_tree_))
    {
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }

    robot_tree_.getChain(chain_base_link_, chain_tip_link_, robot_.kinematic_chain);
    if (robot_.kinematic_chain.getNrOfJoints() == 0)
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
    for (uint16_t i = 0; i < robot_.kinematic_chain.getNrOfSegments(); i++)
    {
        robot_.joints.push_back(robot_.kinematic_chain.getSegment(i).getJoint());
        ROS_INFO_STREAM("Chain segment "<< robot_.kinematic_chain.getSegment(i).getName());
        F=robot_.kinematic_chain.getSegment(i).getFrameToTip();
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

                KDL::Frame f = robot_.forward_kinematics.at(i-1).getFrameToTip()*robot_.kinematic_chain.getSegment(i).getFrameToTip();
                KDL::Segment link(robot_.kinematic_chain.getSegment(i).getName(),KDL::Joint(KDL::Joint::None),f,KDL::RigidBodyInertia::Zero());
                robot_.forward_kinematics.push_back(link);
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
                    // joint_frames its the same as chain_.getSegment(i).getFrameToTip() which is not in the robot variable
                    // in this case robot_.kinematic_chain.getSegment(i).getFrameToTip()

                }
#ifdef __DEBUG__
                rot=robot_.kinematic_chain.getSegment(i).getFrameToTip().M;
                pos=robot_.kinematic_chain.getSegment(i).getFrameToTip().p;
                ROS_INFO("Rotational joint");
                ROS_INFO_STREAM("Joint name "<< robot_.kinematic_chain.getSegment(i).getJoint().getName());
                ROS_INFO_STREAM("Joint position "<< " X: " << pos.x()<< " Y: " << pos.y()<< " Z: " << pos.z());
                ROS_WARN("Rotation matrix %f %f %f \n %f %f %f \n %f %f %f \n",rot(0,0),rot(0,1),rot(0,2),rot(1,0),rot(1,1),rot(1,2),rot(2,0),rot(2,1),rot(2,2));
                ROS_INFO_STREAM("Joint position of transformation"<< " X: " << pos.x()<< " Y: " << pos.y()<< " Z: " << pos.z());            //F_previous.p= pos;
#endif
            }
        }

}
