/*
 * forward_kinematics.h
 *
 *  Created on: 18.06.2017
 *      Author: chris
 */

#include <cob_nonlinear_mpc/nonlinear_mpc.h>

#ifndef COB_CONTROL_COB_NONLINEAR_MPC_INCLUDE_COB_NONLINEAR_MPC_FORWARD_KINEMATICS_H_
#define COB_CONTROL_COB_NONLINEAR_MPC_INCLUDE_COB_NONLINEAR_MPC_FORWARD_KINEMATICS_H_

class ForwardKinematics
{
    private:

        ros::NodeHandle nh_;
        std::vector<SX> fk_vector_;
        std::vector<SX> fk_base_;

    public:
        ForwardKinematics(){}
        ~ForwardKinematics(){}

        void generate_symbolic_forward_kinematics(Robot& robot, int control_dim, int state_dim);

};

#endif /* COB_CONTROL_COB_NONLINEAR_MPC_INCLUDE_COB_NONLINEAR_MPC_FORWARD_KINEMATICS_H_ */
