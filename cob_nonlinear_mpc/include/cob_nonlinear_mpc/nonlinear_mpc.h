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

#ifndef NONLINEAR_MPC_H
#define NONLINEAR_MPC_H

#include <ros/ros.h>

#include <stdlib.h>

using namespace std;

class MPC
{
private:

    int num_shooting_nodes_;
    double time_horizon_;
    int state_dim_;
    int control_dim_;

    // PATH CONSTRAINTS
    vector<double> state_path_constraints_min_, state_path_constraints_max_;
    vector<double> state_terminal_constraints_min_, state_terminal_constraints_max_;
    vector<double> input_constraints_min_, input_constraints_max_;


public:
    MPC(int num_shooting_nodes,double time_horizon ,int state_dim,int control_dim)
    {
        num_shooting_nodes_=num_shooting_nodes;
        time_horizon_=time_horizon;
        state_dim_=state_dim;
        control_dim_=control_dim;
    }
    ~MPC(){}

    int get_num_shooting_nodes();

    double get_time_horizon();

    int get_state_dim();

    int get_control_dim();

    void set_path_constraints(vector<double> state_path_constraints_min,vector<double> state_path_constraints_max);

    void set_state_constraints(vector<double> state_terminal_constraints_min,vector<double> state_terminal_constraints_max);

    void set_input_constraints(vector<double> input_constraints_min,vector<double> input_constraints_max);

};

#endif  // NONLINEAR_MPC_H
