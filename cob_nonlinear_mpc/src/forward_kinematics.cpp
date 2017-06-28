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
 * \date Date of creation: July, 2017
 *
 * \brief
 *
 *
 ****************************************************************/
#include <cob_nonlinear_mpc/forward_kinematics.h>

void ForwardKinematics::symbolic_jacobian(Robot &robot){
    SX Jv_i = SX::vertcat({0,0,0});
    SX Jw_i = SX::vertcat({0,0,0});
    SX z0 = SX::vertcat({0,0,1});
    SX o0 = SX::vertcat({0,0,0});
    int end = fk_vector_.size();
    for(int i=0;i<robot.joint_frames.size();i++)
    {
        KDL::Vector pos;
        KDL::Rotation rot;
        rot=robot.joint_frames.at(i).M;
        pos=robot.joint_frames.at(i).p;

        if(i==0){
            if(robot.kinematic_chain.getSegment(i).getJoint().getType()==0){ //ROTATIONAL JOINT
                SX T_0_end = SX::vertcat({fk_vector_.at(end)(0,3),fk_vector_.at(end)(1,3),fk_vector_.at(end)(2,3)});
                Jv_i = SX::cross(z0,T_0_end);
                Jw_i=z0;
            }
            else{ // PRISMATIC JOINT
                Jv_i = z0;
                Jw_i=  SX::vertcat({0,0,0});
            }
        }
        else{
            if(robot.kinematic_chain.getSegment(i).getJoint().getType()==0){ //ROTATIONAL JOINT
                SX T_0_end = SX::vertcat({fk_vector_.at(end)(0,3),fk_vector_.at(end)(1,3),fk_vector_.at(end)(2,3)});
                SX T_0_i = SX::vertcat({fk_vector_.at(i)(0,3),fk_vector_.at(i)(1,3),fk_vector_.at(i)(2,3)});
                Jv_i = SX::cross(T_0_i,T_0_end-T_0_i);
                Jw_i=SX::vertcat({fk_vector_.at(i)(0,2),fk_vector_.at(i)(1,2),fk_vector_.at(i)(2,2)});;
            }
            else{ // PRISMATIC JOINT
                Jv_i = SX::vertcat({fk_vector_.at(i)(0,2),fk_vector_.at(i)(1,2),fk_vector_.at(i)(2,2)});;;
                Jw_i=  SX::vertcat({0,0,0});
            }
        }
    }
}

void ForwardKinematics::symbolic_fk(Robot &robot)
{
    // Casadi symbolics

    SX T = SX::sym("T",4,4);
    if(robot.base_active_)
    {
    ////generic rotation matrix around z and translation vector for x and y
        T(0,0) = cos(x_(2)); T(0,1) = -sin(x_(2));  T(0,2) = 0.0; T(0,3) = x_(0);
        T(1,0) = sin(x_(2)); T(1,1) = cos(x_(2));   T(1,2) = 0.0; T(1,3) = x_(1);
        T(2,0) = 0.0;        T(2,1) = 0.0;          T(2,2) = 1.0; T(2,3) = 0;
        T(3,0) = 0.0;        T(3,1) = 0.0;          T(3,2) = 0.0; T(3,3) = 1.0;
        T_base_ = T; //
    }
    int offset;

    for(int i=0;i<robot.joint_frames.size();i++)
    {
        KDL::Vector pos;
        KDL::Rotation rot;
        rot=robot.joint_frames.at(i).M;
        pos=robot.joint_frames.at(i).p;
        robot.joints.at(i).getTypeName();
        if(robot.base_active_)
        { // if base active first initial control variable belong to the base
        // here each joint is considered to be revolute.. code needs to be updated for prismatic
        //rotation matrix of the joint * homogenic transformation matrix of the next joint relative to the previous
        //still needs to be improved... if the chain is composed by joint than not joint than joint again this is going to be wrong
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

        T_BVH fk_transform;

        // Todo: There must be a better solution..
        fk_transform.link = robot.kinematic_chain.getSegment(robot.forward_kinematics.size() - robot.joint_frames.size() +i).getName();

        fk_transform.T = T;

        if(pos.z() > 0.1)
        {
            T(2,3) = T(2,3)/2;
            fk_transform.constraint = true;
        }
        else
        {
            fk_transform.constraint = false;
        }
        fk_transform.BVH_p = T;

        transformation_vector_.push_back(fk_transform);
    }

    // Get Endeffector FK
    SX tmp;

    for(int i=0; i < transformation_vector_.size(); i++)
    {

        if(robot.base_active_)
        {
            if(i==0)
            {
                ROS_WARN("BASE IS ACTIVE");
                tmp = mtimes(T_base_,transformation_vector_.at(i).T);
            }
            else
            {
                tmp = mtimes(tmp,transformation_vector_.at(i).T);
            }
        }
        else
        {
            if(i==0)
            {
                tmp = transformation_vector_.at(i).T;
            }
            else
            {
                tmp = mtimes(tmp,transformation_vector_.at(i).T);
            }
        }
        fk_vector_.push_back(tmp); // stacks up multiplied transformation until link n
    }
    ROS_INFO("Done with forward kinemtcis...");
}

std::vector<SX> ForwardKinematics::getFkVector()
{
    return fk_vector_;
}

SX ForwardKinematics::getFkBase()
{
    return T_base_;
}

std::vector<T_BVH> ForwardKinematics::getTransformVector()
{
    return transformation_vector_;
}


void ForwardKinematics::setX(SX x)
{
    x_ = x;
}

void ForwardKinematics::setU(SX u)
{
    u_ = u;
}

SX ForwardKinematics::getX()
{
    return x_;
}

SX ForwardKinematics::getU()
{
    return u_;
}
