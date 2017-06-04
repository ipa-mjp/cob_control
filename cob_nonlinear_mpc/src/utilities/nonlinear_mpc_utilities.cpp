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

void CobNonlinearMPC::generate_symbolic_forward_kinematics(){
    // Casadi symbolics
        u_ = SX::sym("u", control_dim_);  // control
        x_ = SX::sym("x", state_dim_); // states

        SX T = SX::sym("T",4,4);
        if(base_active_){
            ////generic rotation matrix around z and translation vector for x and y
            T(0,0) = cos(x_(2)); T(0,1) = -sin(x_(2));  T(0,2) = 0.0; T(0,3) = x_(0);
            T(1,0) = sin(x_(2)); T(1,1) = cos(x_(2));   T(1,2) = 0.0; T(1,3) = x_(1);
            T(2,0) = 0.0;        T(2,1) = 0.0;          T(2,2) = 1.0; T(2,3) = 0;
            T(3,0) = 0.0;        T(3,1) = 0.0;          T(3,2) = 0.0; T(3,3) = 1.0;
            fk_base_ = T; //Base forward kinematics
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
            if(base_active_){ // if base active first initial control variable belong to the base
                // here each joint is considered to be revolute.. code needs to be updated for prismatic
                //rotation matrix of the joint * homogenic transformation matrix of the next joint relative to the previous
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
                {
                    ROS_WARN("BASE IS ACTIVE");
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
}

void CobNonlinearMPC::generate_bounding_volumes(){

}
