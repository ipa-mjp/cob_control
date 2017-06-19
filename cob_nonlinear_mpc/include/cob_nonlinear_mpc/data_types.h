/*
 * data_types.h
 *
 *  Created on: 18.06.2017
 *      Author: chris
 */

#ifndef COB_CONTROL_COB_NONLINEAR_MPC_INCLUDE_COB_NONLINEAR_MPC_DATA_TYPES_H_
#define COB_CONTROL_COB_NONLINEAR_MPC_INCLUDE_COB_NONLINEAR_MPC_DATA_TYPES_H_

using namespace casadi;
using namespace std;

struct T_BVH
{
    SX T;
    SX BVH_p;
    string link;
    bool constraint = false;
};


#endif /* COB_CONTROL_COB_NONLINEAR_MPC_INCLUDE_COB_NONLINEAR_MPC_DATA_TYPES_H_ */
