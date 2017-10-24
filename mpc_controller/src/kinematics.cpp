
#include <mpc_controller/kinematics.h>

using namespace nmpc;

//--------------------------------------------------------------------------------------------------------------------------------------------------------------

Kinematics::Kinematics(const std::string rbt_description, const std::string& chain_base_link, const std::string& chain_tip_link )
{

	this->chain_base_link_ = chain_base_link	;
	this->chain_tip_link_  = chain_tip_link 	;

	//tree from parameter server
    if (!kdl_parser::treeFromParam("/robot_description", this->robot_tree_))
        ROS_ERROR("Failed to construct kdl tree");

    //kinematic chain
    this->robot_tree_.getChain( chain_base_link, chain_tip_link, this->kinematic_chain_ );

    //cross check kinematic chain properly initialize
    if ( this->kinematic_chain_.getNrOfJoints() == EMPTY)
    	ROS_ERROR("Failed to construct kinematic chain");

    //urdf model
    if ( !this->robot_urdf_model_.initParam(rbt_description) )
    	ROS_ERROR("Failed to parse urdf file for JointLimits");

#ifdef __DEBUG__
        ROS_INFO("Number of joints: %ld", this->kinematic_chain_.getNrOfJoints());
        ROS_INFO("Number of segments: %ld", this->kinematic_chain_.getNrOfSegments());
#endif

    this->dof_ = this->kinematic_chain_.getNrOfJoints();

    this->nr_segments_ = this->kinematic_chain_.getNrOfSegments();


    for (unsigned int i = 0; i < this->nr_segments_; ++i)
    {
    	this->joints_.push_back( this->kinematic_chain_.getSegment(i).getJoint());

    	this->joints_name_.push_back( this->joints_.at(i).getName());

    	this->joints_type_.push_back( this->joints_.at(i).getTypeName());

    }
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------

Kinematics::Kinematics( const KDL::Chain& kinematics_chain, const std::string& chain_base_link, const std::string& chain_tip_link )
{

	this->chain_base_link_ = chain_base_link	;
	this->chain_tip_link_  = chain_tip_link 	;

	this->kinematic_chain_ = kinematics_chain;

    this->dof_ = this->kinematic_chain_.getNrOfJoints();

    this->nr_segments_ = this->kinematic_chain_.getNrOfSegments();

    this->joints_name_.resize( this->dof_);

    this->joints_frame_.resize( this->dof_);

}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------

Kinematics::Kinematics( const KDL::Tree& robot_kdl_tree, const std::string& chain_base_link, const std::string& chain_tip_link )
{

	this->chain_base_link_ = chain_base_link	;
	this->chain_tip_link_  = chain_tip_link 	;

	this->robot_tree_ = robot_kdl_tree;

    robot_tree_.getChain( chain_base_link, chain_tip_link, this->kinematic_chain_ );

    this->dof_ = this->kinematic_chain_.getNrOfJoints();

    this->nr_segments_ = this->kinematic_chain_.getNrOfSegments();

    this->joints_name_.resize( this->dof_);

    this->joints_frame_.resize( this->dof_);

}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------

std::vector<std::string> Kinematics::getJntNames()
{
	return	this->joints_name_;
}

std::vector<std::string> Kinematics::getJntTypes()
{
	return	this->joints_type_;
}

std::vector<KDL::Frame> Kinematics::getJntFrames()
{
	return	this->joints_frame_;
}

std::string Kinematics::getRootlinkName()
{
	return this->chain_base_link_;
}

std::string Kinematics::getTiplinkName()
{
	return this->chain_tip_link_;
}

unsigned int Kinematics::getDOF()
{
	return this->dof_;
}

unsigned int Kinematics::getNrOfSegments()
{
	return this->nr_segments_;
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------
/*
void Kinematics::setJntNames(const std::vector<KDL::Joint>& jntName)
{
	for (unsigned int i = 0; i < jntName.size(); ++i)
	{
		this->joints_[i] = jntName[i];
	}
}*/

void Kinematics::setJntNames(const std::vector<std::string>& jntName)
{
	for (unsigned int i = 0; i < jntName.size(); ++i)
	{
		this->joints_name_[i] = jntName[i];
	}
}

void Kinematics::setJntFrames(const std::vector<KDL::Frame>& jntFrame)
{
	for (unsigned int i = 0; i < jntFrame.size(); ++i)
	{
		this->joints_frame_[i] = jntFrame[i];
	}
}

void Kinematics::setRootlinkName(const std::string& base_link)
{
	this->chain_base_link_ = base_link;
}

void Kinematics::setTiplinkName(const std::string& tip_link)
{
	this->chain_tip_link_ =tip_link;
}

void Kinematics::setDOF(const unsigned int& dof)
{
	this->dof_ = dof;
}

void Kinematics::setNrOfSegments(const unsigned int& nr_segment)
{
	this->nr_segments_ = nr_segment;
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------

//todo: look implementation of base active or not
bool Kinematics::isBaseActive()
{
	return false;
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------

template<class T>
void Kinematics::printVector(const T& vec)
{
	std::vector<T> lcl_vec;

	for(auto it = lcl_vec.begin(); it!= lcl_vec.end(); ++it)
		std::cout<<"\033[36;1m"<<*it<<"\033[36;0m"<<std::endl;
}

/*
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
void Kinematics::forward_kinematics( const std::vector<double>& q , const std::string& chain_base_link, const std::string& chain_tip_link, const std::string& root_frame=" " )
{
	//Eigen::Matrix lcl_homo_mat;

	for (unsigned int i = 0; i < this->nr_segments_; i++)
	{

		if ( this->joints_.at(i).getType() == 8)	//fixed joint
		{


		}




	}



}
*/

Eigen::Matrix3Xd Kinematics::create_rotation_matrix( const std::vector<double>& angle, const std::vector<uint16_t>& axis)
{
	Eigen::Matrix3Xd homo_matrix;
	homo_matrix = Eigen::Matrix3Xd::Identity(3,3);

	if (axis == std::vector<uint16_t> {1,0,0})
	{
		std::cout<<"\033[36;1m"<<"x axis"<<"\033[36;0m"<<std::endl;
		homo_matrix(1,1) = std::cos(angle[0]);
		homo_matrix(1,2) = (-1) * std::sin(angle[0]);
		homo_matrix(2,1) = std::sin(angle[0]);
		homo_matrix(2,2) = std::cos(angle[0]);
	}
	else if (axis == std::vector<uint16_t> {0,1,0})
	{
		std::cout<<"\033[36;1m"<<"y axis"<<"\033[36;0m"<<std::endl;
		homo_matrix(0,0) = std::cos(angle[1]);
		homo_matrix(0,2) = std::sin(angle[1]);
		homo_matrix(2,0) = (-1) * std::sin(angle[1]);
		homo_matrix(2,2) = std::cos(angle[1]);
	}
	else if (axis == std::vector<uint16_t> {0,0,1})
	{
		std::cout<<"\033[36;1m"<<"z axis"<<"\033[36;0m"<<std::endl;
		homo_matrix(0,0) = std::cos(angle[2]);
		homo_matrix(0,1) = std::sin(angle[2]);
		homo_matrix(1,0) = (-1) * std::sin(angle[2]);
		homo_matrix(1,1) = std::cos(angle[2]);
	}

	return homo_matrix;
}

Eigen::Matrix3Xd Kinematics::create_rotation_matrix( const double& angle, const std::string& axis )
{
	Eigen::Matrix3Xd homo_matrix;
	homo_matrix = Eigen::Matrix3Xd::Identity(3,3);

	if (axis == "x")
	{
		std::cout<<"\033[36;1m"<<"x axis"<<"\033[36;0m"<<std::endl;
		homo_matrix(1,1) = std::cos(angle);
		homo_matrix(1,2) = (-1) * std::sin(angle);
		homo_matrix(2,1) = std::sin(angle);
		homo_matrix(2,2) = std::cos(angle);
	}
	else if (axis == "y")
	{
		std::cout<<"\033[36;1m"<<"y axis"<<"\033[36;0m"<<std::endl;
		homo_matrix(0,0) = std::cos(angle);
		homo_matrix(0,2) = std::sin(angle);
		homo_matrix(2,0) = (-1) * std::sin(angle);
		homo_matrix(2,2) = std::cos(angle);
	}
	else if (axis == "z")
	{
		std::cout<<"\033[36;1m"<<"z axis"<<"\033[36;0m"<<std::endl;
		homo_matrix(0,0) = std::cos(angle);
		homo_matrix(0,1) = std::sin(angle);
		homo_matrix(1,0) = (-1) * std::sin(angle);
		homo_matrix(1,1) = std::cos(angle);
	}

	return homo_matrix;

}

bool operator==(std::vector<uint16_t> vec1, std::vector<uint16_t> vec2)
{
	for (uint16_t i = 0; i < vec1.size(); ++i)
	{
		if (vec1[i] != vec2[i])
			return false;
	}
return true;
}


void Kinematics::tf_listener_function( geometry_msgs::TransformStamped& transform_msg, const std::string& source_frame, const std::string& target_frame)
{
	tf::TransformListener listener;
	 tf::StampedTransform transform;
	try
	{
	    listener.waitForTransform( target_frame, source_frame, ros::Time(0), ros::Duration(10.0) );
	    listener.lookupTransform( target_frame, source_frame, ros::Time(0), transform);
	    tf::transformStampedTFToMsg(transform, transform_msg);
	}
	catch (tf::TransformException e)
	{
		ROS_ERROR("Kinematics::tf_listener_function -- Filed to find tf transform: %s",e.what());
		ros::Duration(1.0).sleep();
	}
}

//todo: if transformation wrong than need to check about rotation
void Kinematics::tf_listener_function( geometry_msgs::PoseStamped& poseStamp, const std::string& source_frame, const std::string& target_frame )
{
	tf::TransformListener listener;
	tf::StampedTransform transform;
	geometry_msgs::TransformStamped transform_msg;
	try
	{
	    listener.waitForTransform( target_frame, source_frame, ros::Time(0), ros::Duration(10.0) );
	    listener.lookupTransform( target_frame, source_frame, ros::Time(0), transform);
	    tf::transformStampedTFToMsg(transform, transform_msg);

	    poseStamp.pose.position.x = transform_msg.transform.translation.x;
	    poseStamp.pose.position.y = transform_msg.transform.translation.y;
	    poseStamp.pose.position.z = transform_msg.transform.translation.z;
	    poseStamp.pose.orientation = transform_msg.transform.rotation;

	    poseStamp.header = transform_msg.header;
	    ros::Duration(0.01).sleep();
	}
	catch (tf::TransformException e)
	{
		ROS_ERROR("Kinematics::tf_listener_function -- Filed to find tf transform: %s",e.what());
		ros::Duration(1.0).sleep();
	}
}

void Kinematics::tf_listener_function( std::vector<double>& angle, const std::string& source_frame=" ", const std::string& target_frame=" " )
{
	geometry_msgs::PoseStamped poseStamp;
	tf::Quaternion tf_quternion;

	angle.resize(3, 0.0);

	this->tf_listener_function(poseStamp, source_frame, target_frame);

	tf::quaternionMsgToTF(poseStamp.pose.orientation, tf_quternion);

	tf::Matrix3x3(tf_quternion).getRPY(angle[0], angle[1], angle[2]);

	/*
	this->tf_listener_function(transform_msgs, source_frame, target_frame);
	btQuaternion q(transform_msgs.transform.rotation.x, transform_msgs.transform.rotation.y, transform_msgs.transform.rotation.z, transform_msgs.transform.rotation.w);
	double roll, pitch, yaw;
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
*/
}
