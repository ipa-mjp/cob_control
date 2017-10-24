
#include <mpc_controller/kinematics.h>

using namespace nmpc;

//--------------------------------------------------------------------------------------------------------------------------------------------------------------

Kinematics::Kinematics(const std::string& rbt_description, const std::string& chain_base_link, const std::string& chain_tip_link )
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

    this->dof_ = this->kinematic_chain_.getNrOfJoints();

    this->nr_segments_ = this->kinematic_chain_.getNrOfSegments();


    for (unsigned int i = 0; i < this->nr_segments_; ++i)
    {
    	this->joints_.push_back( this->kinematic_chain_.getSegment(i).getJoint());

    	this->joints_name_.push_back( this->joints_.at(i).getName());

    	this->joints_type_.push_back( this->joints_.at(i).getTypeName());

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

	std::cout<<"\033[92m"<<"###########  homogenous matrix ######### "<<"\033[0m"<<std::endl;
	this->homoMatrixAtEachJoint();

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
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
void Kinematics::forwardKinematics( const std::vector<double>& q , const std::string& chain_base_link, const std::string& chain_tip_link, const std::string& root_frame=" " )
{
	//Eigen::Matrix lcl_homo_mat;

	for (unsigned int i = 0; i < this->joints_frame_.size(); i++)
	{

		KDL::Vector position;
		KDL::Rotation rotation;
		std::string jntType;

		rotation = this->joints_frame_.at(i).M			;
		position = this->joints_frame_.at(i).p			;
		jntType  = this->joints_.at(i).getTypeName()	;

		Eigen::Matrix4d homo_mat;
		homo_mat = Eigen::Matrix3Xd::Identity(3,3);

		//homo_mat(0,0) = rotation(0,0) * std::cos()


	}
}*/

void Kinematics::homoMatrixAtEachJoint()
{

	KDL::Frame kdl_frame;
	double roll, pitch, yaw;


	for (uint16_t i = 0; i < this->nr_segments_; ++i)
	{
		//ROS_INFO_STREAM("Chain segment "<< this->kinematic_chain_.getSegment(i).getName());

		kdl_frame = this->kinematic_chain_.getSegment(i).getFrameToTip();

		kdl_frame.M.GetRPY(roll, pitch, yaw);

		std::cout<<"\033[36;1m"<<"px: "<<kdl_frame.p.x()<<"  py: "<<kdl_frame.p.y()<<"  pz: "<<kdl_frame.p.z()<<"\033[36;0m"<<std::endl;
		std::cout<<"\033[36;1m"<<"roll: "<<roll<<"  pitch: "<<pitch<<"  yaw: "<<yaw<<"\033[36;0m"<<std::endl;

	}

}

void Kinematics::computeForwardKinematics()
{
	this->homoMatrixAtEachJoint();

}

