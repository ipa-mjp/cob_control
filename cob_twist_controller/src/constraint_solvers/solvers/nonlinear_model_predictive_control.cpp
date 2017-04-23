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
 *   ROS package name: cob_twist_controller
 *
 * \author
 *   Author: Christoph Mark, email: christoph.mark@ipa.fraunhofer.de
 *
 * \date Date of creation: April, 2017
 *
 * \brief
 *   This header contains the interface description of constraint solvers
 *   Pure virtual methods have to be implemented in subclasses
 *
 ****************************************************************/

#include "cob_twist_controller/constraint_solvers/solvers/nonlinear_model_predictive_control.h"

using namespace std;

Eigen::MatrixXd NonlinearModelPredictiveControl::solve(const Vector6d_t& in_cart_velocities,
                                                          const JointStates& joint_states)
{


    uint32_t cols = this->jacobian_data_.cols();
    Eigen::VectorXd weighting = Eigen::VectorXd::Ones(cols);
    return weighting.asDiagonal();
}


Eigen::MatrixXd NonlinearModelPredictiveControl::calculateWeighting(const Vector6d_t& in_cart_velocities, const JointStates& joint_states) const
{
    uint32_t cols = this->jacobian_data_.cols();
    Eigen::VectorXd weighting = Eigen::VectorXd::Ones(cols);
    return weighting.asDiagonal();
}
