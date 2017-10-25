
#include <mpc_controller/kinematics.h>

using namespace nmpc;

//--------------------------------------------------------------------------------------------------------------------------------------------------------------

Kinematics::Kinematics(const std::string& rbt_description, const std::string& chain_base_link, const std::string& chain_tip_link, const std::string& root_frame )
{

	this->chain_base_link_ = chain_base_link	;
	this->chain_tip_link_  = chain_tip_link 	;
	this->root_frame_ 	   = root_frame			;

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

    this->dof_ = this->kinematic_chain_.getNrOfJoints();

    this->nr_segments_ = this->kinematic_chain_.getNrOfSegments();

    this->joint_axis_.resize(this->nr_segments_);

    for (unsigned int i = 0; i < this->nr_segments_; ++i)
    {
    	this->joints_.push_back( this->kinematic_chain_.getSegment(i).getJoint());

    	this->joints_name_.push_back( this->joints_.at(i).getName());

    	if (this->joints_.at(i).getType() == revolute)
    		this->joints_type_.push_back ( "revolute" );

    	else if (this->joints_.at(i).getType() == fixed)
    		this->joints_type_.push_back ( "fixed" );

    	//for every segment joint axis store in vector
    	this->joint_axis_[i].push_back( this->joints_.at(i).JointAxis().x());
    	this->joint_axis_[i].push_back( this->joints_.at(i).JointAxis().y());
    	this->joint_axis_[i].push_back( this->joints_.at(i).JointAxis().z());

    	//gives homo matrix at each joints
    	//this->joints_frame_.push_back( this->kinematic_chain_.getSegment(i).getFrameToTip() );

    	this->joints_mass_.push_back( this->kinematic_chain_.getSegment(i).getInertia().getMass() );

    	//segments
    	std::vector<KDL::Frame> last_frame;
		if (last_frame.empty())
		{
			last_frame.push_back( this->kinematic_chain_.getSegment(i).getFrameToTip() );
		}
		else
		{
			last_frame.push_back( last_frame.at(i-1) * this->kinematic_chain_.getSegment(i).getFrameToTip());
		}

		if (this->joints_type_.at(i) == "fixed")
		{
			KDL::Segment link( this->kinematic_chain_.getSegment(i).getName(), KDL::Joint(KDL::Joint::None), last_frame.at(i), KDL::RigidBodyInertia::Zero());
			this->kdl_segments_.push_back(link);
		}

		//todo consider all axis
		else if (this->joints_type_.at(i) == "revolute")
		{
			KDL::Segment link( this->kinematic_chain_.getSegment(i).getName(), KDL::Joint(KDL::Joint::RotZ), last_frame.at(i), KDL::RigidBodyInertia::Zero());
			this->kdl_segments_.push_back(link);

			if (this->joints_frame_.empty())
			{
				this->joints_frame_.push_back( last_frame.at(i) );
			}
			else
			{
				this->joints_frame_.push_back( this->kinematic_chain_.getSegment(i).getFrameToTip() );
			}
		}
    }
}

void Kinematics::debugCodeFunctionality(void)
{

	std::cout<<"\033[92m"<<"###########  Check constructor values ######### "	<<"\033[0m"<<std::endl;
	std::cout<<"\033[36;1m"<<"Chain_base_link_: "	<< this->chain_base_link_ 	<<"\033[36;0m"<<std::endl;
	std::cout<<"\033[36;1m"<<"Chain_tip_link_: "	<< this->chain_tip_link_ 	<<"\033[36;0m"<<std::endl;
	std::cout<<"\033[36;1m"<<"DOF: "				<< this->dof_ 				<<"\033[36;0m"<<std::endl;
	std::cout<<"\033[36;1m"<<"Nr_segments: "		<< this->nr_segments_ 		<<"\033[36;0m"<<std::endl;
	std::cout<<"\033[36;1m"<<"Joint name: "			<< this->joints_name_ 		<<"\033[36;0m"<<std::endl;
	std::cout<<"\033[36;1m"<<"Joint type: "			<< this->joints_type_ 		<<"\033[36;0m"<<std::endl;
	std::cout<<"\033[36;1m"<<"Base_active: "		<< this->base_active_ 		<<"\033[36;0m"<<std::endl;

	std::cout<<"\033[36;1m"<<"Joints: "	<<"\033[36;0m"<<std::endl;
	for (std::vector<KDL::Joint>::const_iterator it = this->joints_.begin(); it!= joints_.end(); ++it)
	{
		//for (unsigned int i = 0; i < it->)
		std::cout<<"\033[30;1m"	<<" joint axis: "	<< it->JointAxis().x() << " "<< it->JointAxis().y() << " "<< it->JointAxis().z()
								<<", jnt name: "	<< it->getName()
								<<", jnt type: "	<< it->getType()
								<<", jnt name: "	<< it->getTypeName()
								<<" jnt origin: "	<< it->JointOrigin().x()<< " "<< it->JointOrigin().y() << " "<< it->JointOrigin().z()

				<<"\033[30;0m"<<std::endl;
	}

	std::cout<<"\033[36;1m"<<"Joints frames: "	<<"\033[36;0m"<<std::endl;
	for (std::vector<KDL::Frame>::const_iterator it = this->joints_frame_.begin(); it!= joints_frame_.end(); ++it)
	{
		KDL::Rotation rot_mat = it->M;
		KDL::Vector pos_mat = it->p;

		//for (unsigned int i = 0; i < it->)
		std::cout<<"\033[32;1m"	<<" joint axis: "	<<	" rxx "<< rot_mat(0,0) <<	" rxy "<< rot_mat(0,1) <<	" rxz "<< rot_mat(0,2)
													<<	" ryx "<< rot_mat(1,0) <<	" ryy "<< rot_mat(1,1) <<	" ryz "<< rot_mat(1,2)
													<<	" rzx "<< rot_mat(2,0) <<	" rzy "<< rot_mat(2,1) <<	" rzz "<< rot_mat(2,2)
													<<	" px "<< pos_mat.x() <<	" py "<< pos_mat.y() <<	" pz "<< pos_mat.z()
				<<"\033[32;0m"<<std::endl;
	}

	std::cout<<"\033[92m"<<"###########  homogenous matrix ######### "<<"\033[0m"<<std::endl;
	//this->forwardKinematics();

	std::cout<<"\033[92m"<<"###########  Check set functionality ######### "<<"\033[0m"<<std::endl;
	std::string str;
	std::cout << "Name of base link: ";
	std::cin >> str;
	this->setRootlinkName(str);

	std::cout << "Name of tip link: ";
	std::cin >> str;
	this->setTiplinkName(str);
/*
	std::cout << "set DOF: ";
	std::cin >> str;
	this->setDOF(int(str));

	std::cout << "set segment: ";
	std::cin >> str;
	this->setDOF(int(str));
*/
	//std::cout<<"\033[36;1m"<<"Joint name: "			<< this->getJntNames() 		<<"\033[36;0m"<<std::endl;
	//std::cout<<"\033[36;1m"<<"Joint type: "			<< this->getJntTypes() 		<<"\033[36;0m"<<std::endl;

	std::cout<<std::endl;
	std::cout<<"\033[92m"  <<"###########  Check get functionality ######### "	<<"\033[0m"<<std::endl;
	std::cout<<"\033[36;1m"<<"Chain_base_link_: "	<< this->getRootlinkName() 	<<"\033[36;0m"<<std::endl;
	std::cout<<"\033[36;1m"<<"Chain_tip_link_: "	<< this->getTiplinkName() 	<<"\033[36;0m"<<std::endl;
	std::cout<<"\033[36;1m"<<"DOF: "				<< this->getDOF() 			<<"\033[36;0m"<<std::endl;
	std::cout<<"\033[36;1m"<<"Nr_segments: "		<< this->getNrOfSegments() 	<<"\033[36;0m"<<std::endl;

	std::vector<std::string> vec;
	vec = this->getJntNames();
	std::cout<<"\033[36;1m"<<"Joint name: "			<< vec 						<<"\033[36;0m"<<std::endl;
	vec.clear();
	vec = this->getJntTypes();
	std::cout<<"\033[36;1m"<<"Joint type: "			<< vec 						<<"\033[36;0m"<<std::endl;

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
void Kinematics::forwardKinematics(const std::vector<double>& jntAngle)
{
	for (uint16_t i = 0; i < this->nr_segments_; i++)
	{




	}





}*/

/*
void Kinematics::convertKDLtoEigenMatrix(const KDL::Frame& kdl_mat, Eigen::Matrix4Xd& eig_mat)
{
	eig_mat(0,0) = kdl_mat.M(0,0);	eig_mat(0,1) = kdl_mat.M(0,1);	eig_mat(0,2) = kdl_mat.M(0,2);
	eig_mat(1,0) = kdl_mat.M(1,0);	eig_mat(1,1) = kdl_mat.M(1,1);	eig_mat(1,2) = kdl_mat.M(1,2);
	eig_mat(2,0) = kdl_mat.M(2,0);	eig_mat(2,1) = kdl_mat.M(2,1);	eig_mat(2,2) = kdl_mat.M(2,2);s
}

//be careful angle should be
void Kinematics::createRotationMatrixFromAngle(const double& angle, const uint16_t& nr_seg, Eigen::Matrix4Xd& rot_mat)
{
	//rot_mat = KDL::Rotation::Identity();

	if (this->joint_axis_.at(nr_seg) == std::vector<uint16_t>{ 1,0,0})
	{
		std::cout<<"\033[36;1m"<<"rotation about x-axis"<<"\033[36;0m"<<std::endl;

		rot_mat(1,1) = cos(angle);	rot_mat(1,2) = (-1)*sin(angle);
		rot_mat(2,1) = sin(angle);	rot_mat(2,2) = 		cos(angle);

	}

	else if (this->joint_axis_.at(nr_seg) == std::vector<uint16_t>{ 0,1,0})
	{
		std::cout<<"\033[36;1m"<<"rotation about y-axis"<<"\033[36;0m"<<std::endl;

		rot_mat(0,0) = cos(angle);		rot_mat(0,2) = 		sin(angle);
		rot_mat(2,0) = (-1)*sin(angle);	rot_mat(2,2) = 		cos(angle);

	}
	else if (this->joint_axis_.at(nr_seg) == std::vector<uint16_t>{ 0,0,1})
	{
		std::cout<<"\033[36;1m"<<"rotation about z-axis"<<"\033[36;0m"<<std::endl;

		rot_mat(0,0) = cos(angle);	rot_mat(0,1) = (-1)*sin(angle);
		rot_mat(1,0) = sin(angle);	rot_mat(1,1) = 		cos(angle);
	}

}

//be careful angle should be
void Kinematics::createRotationMatrixFromAngle(const double& angle, const uint16_t& nr_seg, KDL::Frame& rot_mat)
{
	//rot_mat = KDL::Rotation::Identity();

	if (this->joint_axis_.at(nr_seg) == std::vector<uint16_t>{ 1,0,0})
	{
		std::cout<<"\033[36;1m"<<"rotation about x-axis"<<"\033[36;0m"<<std::endl;

		rot_mat(1,1) = cos(angle);	rot_mat(1,2) = (-1)*sin(angle);
		rot_mat(2,1) = sin(angle);	rot_mat(2,2) = 		cos(angle);

	}

	else if (this->joint_axis_.at(nr_seg) == std::vector<uint16_t>{ 0,1,0})
	{
		std::cout<<"\033[36;1m"<<"rotation about y-axis"<<"\033[36;0m"<<std::endl;

		rot_mat(0,0) = cos(angle);		rot_mat(0,2) = 		sin(angle);
		rot_mat(2,0) = (-1)*sin(angle);	rot_mat(2,2) = 		cos(angle);

	}
	else if (this->joint_axis_.at(nr_seg) == std::vector<uint16_t>{ 0,0,1})
	{
		std::cout<<"\033[36;1m"<<"rotation about z-axis"<<"\033[36;0m"<<std::endl;

		rot_mat(0,0) = cos(angle);	rot_mat(0,1) = (-1)*sin(angle);
		rot_mat(1,0) = sin(angle);	rot_mat(1,1) = 		cos(angle);
	}

}
*/

void Kinematics::computeForwardKinematics()
{
;
}

