
#ifndef MPC_CONTROLLER_KINEMATICS_H_
#define MPC_CONTROLLER_KINEMATICS_H_

//ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

//KDL kinematics
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/frames.hpp>

//C++
#include <iostream>
#include <map>
#include <string>

//ACADO
#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>
#include <acado_optimal_control.hpp>
#include <acado/bindings/acado_gnuplot/gnuplot_window.hpp>

/**
 * @brief class for computing forward kinematics and inverse kinematics
 */

namespace nmpc
{

	class Kinematics
	{
	private:

		std::string chain_base_link;
		std::string chain_tip_link;
		std::string root_frame;
		unsigned int segments;
		unsigned int dof;
		std::vector<std::string> jnt_type;
		std::vector<std::vector<uint16_t> > jnt_axis;

		KDL::Chain	kinematic_chain;
		std::vector<KDL::Frame>	frames;	//homo matrix of each frame with prevoius joint
		std::vector<KDL::Joint>	jnts;

	public:

		Kinematics(const std::string rbt_description = "/robot_description", const std::string& chain_base_link="base_link", const std::string& chain_tip_link="gripper", const std::string& root_frame="world");


	};

	#endif	//MPC_CONTROLLER_KINEMATICS_H_
}


