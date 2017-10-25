
#include <mpc_controller/kinematics.h>

using namespace nmpc;

Kinematics::Kinematics(const std::string rbt_description , const std::string& chain_base_link, const std::string& chain_tip_link, const std::string& root_frame)
{
	//base, tip link and root frame
	this->chain_base_link = chain_base_link;
	this->chain_tip_link = chain_tip_link;
	this->root_frame = root_frame;

	//tree from parameter server
	KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromParam(rbt_description, kdl_tree))
        ROS_ERROR("Failed to construct kdl tree");

    //kinematic chain
    kdl_tree.getChain( chain_base_link, chain_tip_link, this->kinematic_chain );

    //segments
    this->segments = this->kinematic_chain.getNrOfSegments();
    this->jnt_axis.resize(this->segments);

    for (uint16_t i = 0; i< this->segments ; ++i)
    {
    	//joints info
    	this->jnts.push_back( this->kinematic_chain.getSegment(i).getJoint());

    	//type of joint
    	if ( this-> jnts.at(i).getType() == 0)
    		this->jnt_type.push_back( "revolute" );

    	if ( this-> jnts.at(i).getType() == 8)
    		this->jnt_type.push_back( "fixed" );

    	else	//todo need to check
    		this->jnt_type.push_back( "prismatic" );

    	//if rot than about axis
    	this->jnt_axis[i].push_back( this->jnts.at(i).JointAxis().x() );
    	this->jnt_axis[i].push_back( this->jnts.at(i).JointAxis().y() );
    	this->jnt_axis[i].push_back( this->jnts.at(i).JointAxis().z() );

    	//frame , homo matrix of each frame
    	this->frames.push_back( this->kinematic_chain.getSegment(i).getFrameToTip() );
    }
}
