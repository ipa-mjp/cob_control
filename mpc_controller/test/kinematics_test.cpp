
#include <ros/ros.h>
#include <mpc_controller/kinematics.h>

using namespace nmpc;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "kinematic_test");
	ros::NodeHandle node_handler;

	if (node_handler.hasParam("/robot_description"))
	{
		Kinematics kinematics("/robot_description","arm_base_link","arm_7_link", "world");
		//std::cout<<"\033[36;1m"<< rbt_des <<"\033[36;0m"<<std::endl;
		//kinematics.debugCodeFunctionality();
	}



	//




}
