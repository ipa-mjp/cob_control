
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
#include <geometry_msgs/TransformStamped.h>
#include <tf2_kdl/tf2_kdl.h>

//KDL kinematics
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/frames.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

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
#define _DEBUG_  true

namespace nmpc
{

	class Kinematics
	{
	private:
		ros::NodeHandle node_handle;
		std::string chain_base_link;
		std::string chain_tip_link;
		std::string root_frame;
		unsigned int segments;
		unsigned int dof;
		std::vector<std::string> jnt_type;
		//std::vector<std::vector<uint16_t> > jnt_axis;
		std::vector<KDL::Vector> jnt_rot_axis;
		std::vector<std::vector<double> > jnt_rot_angle;


		KDL::Chain	kinematic_chain;
		std::vector<KDL::Frame>	frames;
		std::vector<KDL::Joint>	jnts;
		std::vector<KDL::Frame>	jnt_homo_mat;	//homo matrix of each frame with prevoius joint

		std::vector<KDL::Frame> fk_mat;	//ff_mat

		//bool _DEBUG_;
		void printDataMemebers(void);

		void createHomoRoatationMatrix(const uint16_t& seg_nr);
		void createRoatationMatrix(const double& angle, const std::vector<unsigned int>& rot_axis, KDL::Frame& lcl_homo_mat);

	public:

		Kinematics(const std::string rbt_description = "/robot_description", const std::string& chain_base_link="base_link", const std::string& chain_tip_link="gripper", const std::string& root_frame="world");

		void forwardKinematics(const std::vector<double>& jnt_angles);

		void kdl_forwardKinematics(void);

	};

	#endif	//MPC_CONTROLLER_KINEMATICS_H_
}


