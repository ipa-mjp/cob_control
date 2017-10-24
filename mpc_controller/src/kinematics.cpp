
#include <mpc_controller/kinematics.h>

using namespace nmpc;

//--------------------------------------------------------------------------------------------------------------------------------------------------------------

Kinematics::Kinematics(const std::string rbt_description, const std::string& chain_base_link, const std::string& chain_tip_link )
{

	this->chain_base_link_ = chain_base_link	;
	this->chain_tip_link_  = chain_tip_link 	;

    if (!kdl_parser::treeFromParam("/robot_description", robot_tree_))
    {
        ROS_ERROR("Failed to construct kdl tree");
    }

    robot_tree_.getChain( chain_base_link, chain_tip_link, this->kinematic_chain_ );

    this->dof_ = this->kinematic_chain_.getNrOfJoints();

    this->nr_segments_ = this->kinematic_chain_.getNrOfSegments();

    this->joints_name_.resize( this->dof_);

    this->joints_frame_.resize( this->dof_);

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

std::vector<KDL::Joint> Kinematics::getJntNames()
{
	return	this->joints_name_;
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

void Kinematics::setJntNames(const std::vector<KDL::Joint>& jntName)
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
