/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2015 \n
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
 *   Author: Marco Bezzon, email: Marco.Bezzon@ipa.fraunhofer.de
 *
 * \date Date of creation: March, 2015
 *
 * \brief
 *   Implementation of an JLA solver.
 *   Special constraint: Avoid joint limits.
 *
 ****************************************************************/

#include "cob_twist_controller/constraint_solvers/solvers/non_linear_least_square_solver.h"

/**
 * Specific implementation of the solve method using a weighted least norm.
 * This is done by calculation of a weighting which is dependent on inherited classes for the Jacobian.
 * Uses the base implementation of calculatePinvJacobianBySVD to calculate the pseudo-inverse (weighted) Jacobian.
 */
struct CostFunctor {
  template <typename T> bool operator()(const T* const x, T* residual) const {
    residual[0] = 10.0 - x[0];
    return true;
  }
};

Eigen::MatrixXd NonLinearLeastSquareNormSolver::solve(const Vector6d_t& in_cart_velocities,
                                               const JointStates& joint_states)
{
	    // INTRODUCE THE VARIABLES:
	    // ----------------------------
		DifferentialState    x("", 6, 1);    // a differential state vector with dimension 10. (vector)
		DifferentialState    y;
		DifferentialState    z;
	    Control              u("", 7, 1);    // a control input with dimension 2.              (vector)

	    DifferentialEquation f    ;    // the differential equation


	    x.clearStaticCounters();
	    y.clearStaticCounters();
	    z.clearStaticCounters();
	    u.clearStaticCounters();

	    const double t_start =  0.0;
	    const double t_end   =  1.0;
	    const int horizon = 10;


	    // READ A MATRIX "A" FROM A FILE:
	    // ------------------------------
	    DMatrix A;
	    A = this->jacobian_data_;

	    // READ A VECTOR "x0" FROM A FILE:
	    // -------------------------------
	    DVector x0;
	    x0 = in_cart_velocities;

	    DVector xEnd; xEnd = in_cart_velocities * 0;

	    // DEFINE A DIFFERENTIAL EQUATION:
	    // -------------------------------

	    f << dot(x) == A*u;                           			// matrix vector notation for a linear equation
	    f << dot(y) == 0.5*3*u.transpose() * u;			  	// matrix vector notation:  x^x  = scalar product = ||x||_2^2
//	    f << dot(z) == 10000*(x+x0).transpose()*(x+x0);

	    // DEFINE AN OPTIMAL CONTROL PROBLEM:
	    // ----------------------------------
	    OCP ocp( t_start, t_end, horizon );

	    ocp.minimizeMayerTerm( y );		// running cost
//	    ocp.minimizeMayerTerm( z );		// Terminal Cost

	    ocp.subjectTo( f );

	    ocp.subjectTo( AT_START, x == -x0  );
	    ocp.subjectTo( AT_END  , x == xEnd);

	    ocp.subjectTo( AT_START, y == (double)(x0.transpose() * x0));
//	    ocp.subjectTo( AT_START, z == 0);

	    ocp.subjectTo( -M_PI <= u <= M_PI);

	    // DEFINE AN OPTIMIZATION ALGORITHM AND SOLVE THE OCP:
	    // ---------------------------------------------------
	    OptimizationAlgorithm algorithm(ocp);

	    algorithm.set( MAX_NUM_ITERATIONS, 20 );
	    algorithm.set( KKT_TOLERANCE, 1e-10 );

	    algorithm.solve();

	    VariablesGrid states, parameters, controls;

	    algorithm.getControls(controls);


	    return controls.getFirstVector();
}

/**
 * This function returns the identity as weighting matrix for base functionality.
 */
Eigen::MatrixXd NonLinearLeastSquareNormSolver::calculateWeighting(const JointStates& joint_states) const
{
    uint32_t cols = this->jacobian_data_.cols();
    Eigen::VectorXd weighting = Eigen::VectorXd::Ones(cols);
    return weighting.asDiagonal();
}
