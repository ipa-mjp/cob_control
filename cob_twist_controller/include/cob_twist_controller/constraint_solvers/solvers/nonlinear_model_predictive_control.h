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

#ifndef COB_TWIST_CONTROLLER_CONSTRAINT_SOLVERS_SOLVERS_NONLINEAR_MODEL_PREDICTIVE_CONTROL_H
#define COB_TWIST_CONTROLLER_CONSTRAINT_SOLVERS_SOLVERS_NONLINEAR_MODEL_PREDICTIVE_CONTROL_H

#include "cob_twist_controller/cob_twist_controller_data_types.h"
#include "cob_twist_controller/constraint_solvers/solvers/constraint_solver_base.h"

//#include "ceres/ceres.h"
//#include "glog/logging.h"
//#include <acado/acado_toolkit.hpp>
//#include <acado/acado_optimal_control.hpp>
//#include <acado/bindings/acado_gnuplot/gnuplot_window.hpp>

class NonlinearModelPredictiveControl : public ConstraintSolver<>
{
    public:
    NonlinearModelPredictiveControl(const TwistControllerParams& params,
                                           const LimiterParams& limiter_params,
                                           TaskStackController_t& task_stack_controller) :
                ConstraintSolver(params, limiter_params, task_stack_controller)
        {}

        virtual ~NonlinearModelPredictiveControl()
        {}

        virtual Eigen::MatrixXd solve(const Vector6d_t& in_cart_velocities,
                                      const JointStates& joint_states);

    private:

        virtual Eigen::MatrixXd calculateWeighting(const Vector6d_t& in_cart_velocities, const JointStates& joint_states) const;
};

#endif  // COB_TWIST_CONTROLLER_CONSTRAINT_SOLVERS_SOLVERS_NONLINEAR_MODEL_PREDICTIVE_CONTROL_H
