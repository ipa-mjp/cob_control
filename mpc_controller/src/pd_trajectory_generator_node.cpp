#include <ros/ros.h>
#include <ros/package.h>

#include <acado_toolkit.hpp>
#include <acado/acado_optimal_control.hpp>
#include <acado/acado_gnuplot.hpp>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "controller_node");
	ros::NodeHandle nh;

	using namespace ACADO;

	DifferentialState		theta, angular_vel		;	//the differential states
	Control 		  		angular_acc				;	//the control input u
	Parameter				T						;	//the time horizon T
	DifferentialEquation	f						;	//the differential equation
	std::cout<<"\033[36;1m"<<"Hello"<<"\033[36;0m"<<std::endl;
	//----------------------------------------------------------------------

	f	<<	dot(theta) == angular_vel			;	//an implementation
	f	<<	dot(angular_vel) == angular_acc		;	//of the model equations


	//DEFINE LEAST SQUARE FUNCTION
	Function h;

	h << theta;
	h << angular_vel;
	h << angular_acc;


	//LSQ coefficient matrix
	DMatrix Q(5,5);
	Q(0,0) = 10.0;
	Q(1,1) = 10.0;
	Q(2,2) = 1.0;
	Q(3,3) = 1.0;
	Q(4,4) = 1.0e-8;

	//Reference
	DVector ref_vec(5);
	ref_vec.setAll(0.0);


	//DEFINE AN OPTIMAL CONTROL PROBLEM:
	const double tStart = 0.0;
	const double tEnd = 1.0;

	OCP ocp ( tStart, tEnd , 5);
	ocp.minimizeLSQ( Q, h, ref_vec);

	ocp.subjectTo( f )						;
	ocp.subjectTo(	0 <= angular_acc <= 3 )	;

	//SETTING UP THE REAL-TIME ALGORITHM
	//-------------------------------------------------------------------------
	RealTimeAlgorithm alg ( ocp, 0.025);
	alg.set(  MAX_NUM_ITERATIONS, 1 );
	alg.set( PLOT_RESOLUTION, MEDIUM );

	GnuplotWindow window										;	//visualize the results in a
	window.addSubplot( theta, "THETA th")						;	// Gnuplot window.
	window.addSubplot( angular_vel, "VELOCITY omega")			;	// Gnuplot window.
	window.addSubplot( angular_acc, "ACCELERATION m/s^2")		;	// Gnuplot window.

	alg	<< window;

	//	SETUP CONTROLLER AND PERFORM A STEP:
	//------------------------------------------------------------------------
	//StaticReferenceTrajectory zeroReferance ( "/home/bfb-ws/catkin_ws/src/cob_control/mpc_controller/config/ref.txt ");
	VariablesGrid	ref(4, 5);
    ref(0,0 ) = 0.00; ref(1,0 ) = 0.0; ref(2,0 ) = 0.0;	ref(3,0 ) = 0.0;
    ref(0,1 ) = 0.025; ref(1,1 ) = 0.0; ref(2,1 ) = 0.0;	ref(3,1 ) = 0.0;
    ref(0,2 ) = 0.050; ref(1,2 ) = 0.0; ref(2,2 ) = 0.0;	ref(3,2 ) = 0.0;
    ref(0,3 ) = 0.075; ref(1,3 ) = 0.0; ref(2,3 ) = 0.0;	ref(3,3 ) = 0.0;
    ref(0,4 ) = 0.100; ref(1,4 ) = 0.0; ref(2,4 ) = 0.0;	ref(3,4 ) = 0.0;

    StaticReferenceTrajectory zeroReferance (ref);

	Controller controller( alg, zeroReferance );

	DVector y( 4 );
	y.setZero( );
	y(0) = 0.01;

	controller.init( 0.0, y );
	controller.step( 0.0, y );

	std::cout<<"\033[36;1m"<<"Hello"<<"\033[36;0m"<<std::endl;

return 0;
}



