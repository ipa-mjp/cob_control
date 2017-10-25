
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
    if (!kdl_parser::treeFromParam("/robot_description", kdl_tree))
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

    //if debug true than print
    if (_DEBUG_)
    	this->printDataMemebers();

}

void Kinematics::printDataMemebers(void)
{
		std::cout<<"\033[92m"<<"###########  Check constructor values ######### "	<<"\033[0m"<<std::endl;
		std::cout<<"\033[36;1m"<<"Chain_base_link_: "	<< this->chain_base_link 	<<"\033[36;0m"<<std::endl;
		std::cout<<"\033[36;1m"<<"Chain_tip_link_: "	<< this->chain_tip_link 	<<"\033[36;0m"<<std::endl;
		std::cout<<"\033[36;1m"<<"Root frame: "			<< this->root_frame 		<<"\033[36;0m"<<std::endl;
		std::cout<<"\033[36;1m"<<"DOF: "				<< this->dof 				<<"\033[36;0m"<<std::endl;
		std::cout<<"\033[36;1m"<<"Nr_segments: "		<< this-> segments 			<<"\033[36;0m"<<std::endl;


/*
		std::cout<<"\033[36;1m"<<"Joints: "	<<"\033[36;0m"<<std::endl;
		for (std::vector<KDL::Joint>::const_iterator it = this->jnts.begin(); it!= jnts.end(); ++it)
		{
			std::cout<<"\033[30;1m"	<<" joint axis: "	<< it->JointAxis().x() << " "<< it->JointAxis().y() << " "<< it->JointAxis().z()
									<<", jnt name: "	<< it->getName()
									<<", jnt type: "	<< it->getType()
									<<", jnt name: "	<< it->getTypeName()
									<<" jnt origin: "	<< it->JointOrigin().x()<< " "<< it->JointOrigin().y() << " "<< it->JointOrigin().z()

					<<"\033[30;0m"<<std::endl;
		}
*/
		std::cout<<"\033[36;1m"<<"Joints frames: "	<<"\033[36;0m"<<std::endl;
		for (std::vector<KDL::Frame>::const_iterator it = this->frames.begin(); it!= frames.end(); ++it)
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

}
