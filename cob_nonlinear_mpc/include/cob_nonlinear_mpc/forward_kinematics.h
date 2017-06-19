/*
 * forward_kinematics.h
 *
 *  Created on: 18.06.2017
 *      Author: chris
 */

#ifndef FORWARD_KINEMATICS_H
#define FORWARD_KINEMATICS_H
#include <cob_nonlinear_mpc/cob_nonlinear_mpc.h>
#include <cob_nonlinear_mpc/data_types.h>
#include <cob_nonlinear_mpc/robot.h>


using namespace casadi;
using namespace std;



class ForwardKinematics
{
    private:
        ros::NodeHandle nh_;
        std::vector<SX> fk_vector_;
        SX fk_base_;
        std::vector<T_BVH> transformation_vector_; // Vector of symbolic transformation matrices from n-1 to n

    public:
        ForwardKinematics(){}
        ~ForwardKinematics(){}

        void symbolic_fk(Robot &robot, SX u, SX x);
        std::vector<SX> getFkVector();
        SX getFkBase();

};
#endif
