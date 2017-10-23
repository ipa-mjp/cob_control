//ROS
#include <ros/ros.h>

//ACADO
#include <acado_toolkit.hpp>
#include <acado/bindings/acado_gnuplot/gnuplot_window.hpp>

//C++
#include <iostream>
#include <memory>
#include <algorithm>
#include <boost/shared_ptr.hpp>

#include <mpc_controller/kinematics.h>
using namespace nmpc;
//----------------------------------------------------------------------------------------------------------------------------------------------------

/**
 * @brief read parameter from server and files and configure controller parameter, initialize subsriber, publisher
 */

class pm_controller_node_ros
{

private:
	//ROS node Handling
	ros::NodeHandle pmcn_handler_;

	std::string root_frame_, chain_base_link_, chain_tip_link_;

	void getKinematicsParams(void)
	{
		pmcn_handler_.param("root_frame", root_frame_, std::string("world"));
		pmcn_handler_.param("chain_base_link", chain_base_link_, std::string("base_link"));
		pmcn_handler_.param("chain_tip_link", chain_tip_link_, std::string("gripper"));
	}

public:

	pm_controller_node_ros();
	~pm_controller_node_ros();
	bool class_initialization();

};


//---------------------------------------------------------------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
	try
	{
		ros::init(argc, argv, ros::this_node::getName());

		pm_controller_node_ros pm_controller;

		if (!pm_controller.class_initialization())
		{
			ROS_ERROR("Class initialization Failed");
			exit(1);
		}

	}

	catch (ros::Exception &e)
	{
		ROS_ERROR("Error occured: %s ", e.what());
		exit(1);
	}
return 0;
}

