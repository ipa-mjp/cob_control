
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

//ACADO
#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>
#include <acado_optimal_control.hpp>
#include <acado/bindings/acado_gnuplot/gnuplot_window.hpp>


enum
{
	EMPTY,
	SUCCESS,
	FAILURE
};

//------------------------------------------------------------------------------------------------------------------------------------------
/**
 * @brief describe robot model using KDL kinematics
 */
/*
struct RobotModel
{
	unsigned int dof;
    urdf::Model urdf;
    KDL::Chain kinematic_chain;
    std::vector<KDL::Joint> joints;
    std::vector<KDL::Frame> joint_frames;
    std::vector<KDL::Segment> forward_kinematics;
    bool base_active_;
    std::unordered_map<std::string, std::vector<std::string> > self_collision_map_;
    std::string root_frame;
    std::vector<double> masses;

};
*/
//--------------------------------------------------------------------------------------------------------------------------------------------

/**
 * @brief class for computing forward kinematics and inverse kinematics
 */

namespace nmpc
{

	class Kinematics
	{
	private:

		//ROS data member functions
		ros::NodeHandle node_handler_;

		//KDL data member functions
		unsigned int 				dof_				;		// Degree of freedom
		unsigned int 				nr_segments_		;		// Nr of segments
		bool 						base_active_		;		// Mobile or stationary manipulator
		urdf::Model 				robot_urdf_model_	;		// Urdf model of robot
		KDL::Tree					robot_tree_			;		// Robot tree
		KDL::Chain 					kinematic_chain_	;		// Kinematic chain of robot
		std::vector<KDL::Joint> 	joints_				;		// Vector of joint names

		std::vector<KDL::Frame> 	joints_frame_		;		// Vector of joint frame
		std::vector<std::string> 	joints_name_		;
		std::vector<std::string> 	joints_type_		;

		//		std::vector<KDL::Segment>	forward_kinematics_	;		// Number of segments
		std::string 				chain_base_link_	;		// Chain base link
		std::string 				chain_tip_link_		;		// Chain tip link
		std::string 				root_frame_			;

		//Eigen::Matrix 				homo_matrix_		;


		void forward_kinematics(const std::vector<double>& q , const std::string& chain_base_link = " ", const std::string& chain_tip_link = " ", const std::string& root_frame=" " );

		void tf_listener_function( geometry_msgs::TransformStamped& transform_msg, const std::string& source_frame=" ", const std::string& target_frame=" " );

		void tf_listener_function( geometry_msgs::PoseStamped& poseStamp, const std::string& source_frame=" ", const std::string& target_frame=" " );

		void tf_listener_function( std::vector<double>& angle, const std::string& source_frame, const std::string& target_frame );

		Eigen::Matrix3Xd create_rotation_matrix( const std::vector<double>& angle, const std::vector<uint16_t>& axis);

		Eigen::Matrix3Xd create_rotation_matrix( const double& angle, const std::string& axis );

		friend bool operator==(std::vector<uint16_t> vec1, std::vector<uint16_t> vec2);

	public:

		Kinematics();

		/**
		 * @brief Get kinematics chain using parameter is defined by only base_link and tip_link
		 * @param rbt_description = robot description containts urdf of robot
		 * @param chain_base_link = root link of kinematic chain
		 * @param chain_tip_link = tip link of kinematic chain called end-effector link
		 *
		 */
		Kinematics(	const std::string rbt_description = "/robot_description", const std::string& chain_base_link="base_link",	const std::string& chain_tip_link="gripper"	);


		/**
		 * @brief kinematics chain is defined by robot_urdf_model, base_link and tip link
		 * @param robot_urdf_model_ = robot model loaded from urdf file, on parameter server it called robot_description
		 * @param chain_base_link = root link of kinematic chain
		 * @param chain_tip_link = tip link of kinematic chain called end-effector link
		 *
		 */
//		Kinematics(	const urdf::Model& robot_urdf_model_, const std::string& chain_base_link="base_link", const std::string& chain_tip_link="gripper");


		/**
		 * @brief kinematics chain is defined by kinematic_chain, base_link and tip link
		 * @param kinematics_chain_ = initialize by other kinematic chain
		 * @param chain_base_link = root link of kinematic chain
		 * @param chain_tip_link = tip link of kinematic chain called end-effector link
		 *
		 */
		Kinematics(	const KDL::Chain& kinematics_chain, const std::string& chain_base_link="base_link", const std::string& chain_tip_link="gripper" );

		/**
		 * @brief kinematics chain is defined by kinematic_chain, base_link and tip link
		 * @param robot_kdl_tree_ = robot_kdl_tree that read from parameter server
		 * @param chain_base_link = root link of kinematic chain
		 * @param chain_tip_link = tip link of kinematic chain called end-effector link
		 *
		 */
		Kinematics(	const KDL::Tree& robot_kdl_tree, const std::string& chain_base_link="base_link", const std::string& chain_tip_link="gripper" );

		~Kinematics();


		/**
		 * @brief gives name of joints in kinematic chain (between root link and tip link)
		 * @return vector of joint name
		 */
		std::vector<std::string> getJntNames();	//std::vector<KDL::Joint>

		/**
		 * @brief gives type of joints in kinematic chain (between root link and tip link)
		 * @return vector of joint type
		 */
		std::vector<std::string> getJntTypes();

		/**
		 * @brief gives frame of joints in kinematic chain (between root link and tip link)
		 * @return vector of joint frames
		 */
		std::vector<KDL::Frame> getJntFrames();

		/**
		 * @brief gives name of base(root) link in kinematic chain
		 * @return string object of name of base link
		 */
		std::string getRootlinkName();

		/**
		 * @brief gives name of of tip(end-effector) link in kinematic chain
		 * @return string object of name of tip link
		 */
		std::string getTiplinkName();

		/**
		 * @brief gives number of joints in kinematic chain (between root link and tip link) or degree of freedom
		 * @return integer value of dof
		 */
		unsigned int getDOF();

		/**
		 * @brief gives number of joints in kinematic chain (between root link and tip link) or degree of freedom
		 * @return integer value of number of segments
		 */
		unsigned int getNrOfSegments();

		/**
		 * @brief set name of joints in kinematic chain (between root link and tip link)
		 * @param vector of joint name
		 */
		void setJntNames(const std::vector<std::string>& jntName);	//std::vector<KDL::Joint>

		/**
		 * @brief set frame of joints in kinematic chain (between root link and tip link)
		 * @param vector of joint frames
		 */
		void setJntFrames(const std::vector<KDL::Frame>& jntFrame);

		/**
		 * @brief set name of base(root) link in kinematic chain
		 * @param string object of name of base link
		 */
		void setRootlinkName(const std::string& base_link);

		/**
		 * @brief set name of of tip(end-effector) link in kinematic chain
		 * @param string object of name of tip link
		 */
		void setTiplinkName(const std::string& tip_link);

		/**
		 * @brief set number of joints in kinematic chain (between root link and tip link) or degree of freedom
		 * @param integer value of dof
		 */
		void setDOF(const unsigned int& dof);

		/**
		 * @brief set number of joints in kinematic chain (between root link and tip link) or degree of freedom
		 * @param integer value of number of segments
		 */
		void setNrOfSegments(const unsigned int& nr_segment);


		/**
		 * @brief gives info about base is active(mobile robot) or not(stationary platform)
		 * @return true if mobile robot otherwise false
		 */
		bool isBaseActive();

		/**
		 * @brief print any type of vector value for debug purpose
		 *@param vec = template type of vector
		 */
		template<class T>
		void printVector(const T& vec);

	};

	#endif	//MPC_CONTROLLER_KINEMATICS_H_
}


