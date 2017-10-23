#include <ros/ros.h>
#include <ros/package.h>

#include <acado_toolkit.hpp>
#include <acado/acado_optimal_control.hpp>
#include <acado/acado_gnuplot.hpp>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pd_manipulation_node");
	ros::NodeHandle nh;

	using namespace ACADO;

	DifferentialState		s,v,m		;	//the differential states
	Control 		  		u			;	//the control input u
	Parameter				T			;	//the time horizon T
	DifferentialEquation	f( 0.0, T)	;	//the differential equation

	//----------------------------------------------------------------------
	OCP ocp( 0.0, T)					;	//time horizon of the OCP: [0,T]
	ocp.minimizeMayerTerm( T )			;	//the time T should be optimized

	f	<<	dot(s) == v					;	//an implementation
	f	<<	dot(v) == (u-0.2*v*v)/m		;	//of the model equations
	f	<<	dot(m) == -0.01*u*u			;	//for the rocket.

	//set initial values, end values or set equalities constraints
	ocp.subjectTo( f )					;	//minimize T s.t the model,
	ocp.subjectTo( AT_START, s == 0.0 )	;	//the initial values for s,
	ocp.subjectTo( AT_START, v == 0.0 )	;	//the initial values for v,
	ocp.subjectTo( AT_START, m == 1.0 )	;	//the initial values for m,

	ocp.subjectTo( AT_END, s == 10.0 )	;	//the end values for s,
	ocp.subjectTo( AT_END, v == 0.0 )	;	//the end values for vs,

	//set inequalities constraints
	ocp.subjectTo( -0.1 <= v <= 1.7 )	;	//as well as the bounds on v
	ocp.subjectTo( -1.1 <= u <= 1.1 )	;	//as well as the bounds on control input u
	ocp.subjectTo(  5.0 <= T <= 15.0 )	;	//as well as the bounds on the time horizon T

	//--------------------------------------------------------------------------
	GnuplotWindow window				;	//visualize the results in a
	window.addSubplot( s, "DISTANCE s")	;	// Gnuplot window.
	window.addSubplot( v, "VELOCITY v")	;	// Gnuplot window.
	window.addSubplot( m, "MASS m")		;	// Gnuplot window.
	window.addSubplot( u, "CONTROL u")	;	// Gnuplot window.

	//--------------------------------------------------------------------------

	OptimizationAlgorithm algorithm(ocp);	//construct optimization algorithm,

	//algorithm.set( MAX_NUM_ITERATIONS, 20)					;	//set number of iteration as 20.0
	//algorithm.set( HESSIAN_APPROXIMATION, EXACT_HESSIAN )	;	//Approximation of hessian is nothing but exact
	//algorithm.set( HESSIAN_PROJECTION_FACTOR, 2.0)			;	//hessian factor 1.0

	algorithm	<<	window				;	//flush the plot window,
	algorithm.solve()					;	//and solve the problem.

return 0;
}



