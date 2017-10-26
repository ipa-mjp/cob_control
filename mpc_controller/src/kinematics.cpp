
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
    this->jnt_rot_angle.resize(segments);

    //this->jnt_axis.resize(this->segments);

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

    	/*
    	//if rot than about axis
    	this->jnt_axis[i].push_back( this->jnts.at(i).JointAxis().x() );
    	this->jnt_axis[i].push_back( this->jnts.at(i).JointAxis().y() );
    	this->jnt_axis[i].push_back( this->jnts.at(i).JointAxis().z() );*/

    	//frame , homo matrix of each frame
    	this->frames.push_back( this->kinematic_chain.getSegment(i).getFrameToTip() );

    	double roll,pitch,yaw;
    	this->kinematic_chain.getSegment(i).getFrameToTip().M.GetRPY(roll,pitch,yaw);
    	this->jnt_rot_angle.at(i).push_back(roll);	this->jnt_rot_angle.at(i).push_back(pitch);		this->jnt_rot_angle.at(i).push_back(yaw);


    	// rot angle, axis of rotation
    	KDL::Vector rot;
    	this->kinematic_chain.getSegment(i).getFrameToTip().M.GetRotAngle(rot);
    	this->jnt_rot_axis.push_back(rot);

    	this->jnt_homo_mat.push_back(this->frames.at(i));

    	if (this->jnts.at(i).getType() == 0)	//revolute joint	//todo consider test for presmatic joint
    		this->createRoatationMatrix(i);


    	if (_DEBUG_)
    	{
			std::cout<<"\033[36;1m"<<"homo matrix of " << this->jnts.at(i).getName() <<"\033[36;0m"<<std::endl;
			KDL::Rotation rot_mat = this->jnt_homo_mat.at(i).M;
			KDL::Vector pos_mat = this->jnt_homo_mat.at(i).p;

				//for (unsigned int i = 0; i < it->)
				std::cout<<"\033[32;1m"	<<	" rxx "<< rot_mat(0,0) <<	" rxy "<< rot_mat(0,1) <<	" rxz "<< rot_mat(0,2)	<< "\n"
															<<	" ryx "<< rot_mat(1,0) <<	" ryy "<< rot_mat(1,1) <<	" ryz "<< rot_mat(1,2)	<< "\n"
															<<	" rzx "<< rot_mat(2,0) <<	" rzy "<< rot_mat(2,1) <<	" rzz "<< rot_mat(2,2)	<< "\n"
															<<	" px "<< pos_mat.x() <<	" py "<< pos_mat.y() <<	" pz "<< pos_mat.z()
						<<"\033[32;0m"<<std::endl;
    	}

    }

    //if debug true than print
    if (_DEBUG_)
    {
    	//this->printDataMemebers();
    	std::cout<<"\033[20m"<<"###########  fk correctness ######### "	<<"\033[0m"<<std::endl;
		std::vector<double> jnt_angels;
		jnt_angels.resize( 6, 0.0 );
		this->forwardKinematics(jnt_angels);
    }
}

void Kinematics::createRoatationMatrix(const uint16_t& seg_nr)
{

	//x-axis rotation
	double x[3] = {1,0,0};
	if ( (this->jnt_rot_axis.at(seg_nr).x() == 1 || this->jnt_rot_axis.at(seg_nr).x() == -1) && this->jnt_rot_axis.at(seg_nr).y() == 0 && this->jnt_rot_axis.at(seg_nr).z() == 0 )
	{
		double angle = this->jnt_rot_angle.at(seg_nr).at(0);

		if (_DEBUG_)
			std::cout<<"rot about x-axis with angel: "<< angle<<std::endl;

		this->jnt_homo_mat.at(seg_nr).M(0,0) = 1;	this->jnt_homo_mat.at(seg_nr).M(0,1) = 0;			this->jnt_homo_mat.at(seg_nr).M(0,2) = 0;
		this->jnt_homo_mat.at(seg_nr).M(1,0) = 0;	this->jnt_homo_mat.at(seg_nr).M(1,1) = cos(angle);	this->jnt_homo_mat.at(seg_nr).M(1,2) = -1*sin(angle);
		this->jnt_homo_mat.at(seg_nr).M(2,0) = 0;	this->jnt_homo_mat.at(seg_nr).M(2,1) = sin(angle);	this->jnt_homo_mat.at(seg_nr).M(2,2) = cos(angle);

	}

	double y[3] = {0,1,0};
	if ( this->jnt_rot_axis.at(seg_nr).x() == 0 && (this->jnt_rot_axis.at(seg_nr).y() == 1 || this->jnt_rot_axis.at(seg_nr).y() == -1) && this->jnt_rot_axis.at(seg_nr).z() == 0 )
	{
		double angle = this->jnt_rot_angle.at(seg_nr).at(1);

		if (_DEBUG_)
			std::cout<<"rot about y-axis with angel: "<< angle<<std::endl;

		this->jnt_homo_mat.at(seg_nr).M(0,0) = cos(angle);		this->jnt_homo_mat.at(seg_nr).M(0,1) = 0;			this->jnt_homo_mat.at(seg_nr).M(0,2) = sin(angle);
		this->jnt_homo_mat.at(seg_nr).M(1,0) = 0;				this->jnt_homo_mat.at(seg_nr).M(1,1) = 1;			this->jnt_homo_mat.at(seg_nr).M(1,2) = 0;
		this->jnt_homo_mat.at(seg_nr).M(2,0) = -1*sin(angle);	this->jnt_homo_mat.at(seg_nr).M(2,1) = 0;			this->jnt_homo_mat.at(seg_nr).M(2,2) = cos(angle);
	}

	double z[3] = {0,0,1};
	if ( this->jnt_rot_axis.at(seg_nr).x() == 0 && this->jnt_rot_axis.at(seg_nr).y() == 0 && (this->jnt_rot_axis.at(seg_nr).z() == 1 || this->jnt_rot_axis.at(seg_nr).z() == -1) )
	{

		double angle = this->jnt_rot_angle.at(seg_nr).at(2);

		if (_DEBUG_)
			std::cout<<"rot about z-axis with angel: "<< angle<<std::endl;

		this->jnt_homo_mat.at(seg_nr).M(0,0) = cos(angle);	this->jnt_homo_mat.at(seg_nr).M(0,1) = sin(angle);	this->jnt_homo_mat.at(seg_nr).M(0,2) = 0;
		this->jnt_homo_mat.at(seg_nr).M(1,0) = sin(angle);	this->jnt_homo_mat.at(seg_nr).M(1,1) = cos(angle);	this->jnt_homo_mat.at(seg_nr).M(1,2) = 0;
		this->jnt_homo_mat.at(seg_nr).M(2,0) = 0;			this->jnt_homo_mat.at(seg_nr).M(2,1) = 0;			this->jnt_homo_mat.at(seg_nr).M(2,2) = 1;
	}
}

void Kinematics::printDataMemebers(void)
{
		std::cout<<"\033[92m"<<"###########  Check constructor values ######### "	<<"\033[0m"<<std::endl;
		std::cout<<"\033[36;1m"<<"Chain_base_link_: "	<< this->chain_base_link 	<<"\033[36;0m"<<std::endl;
		std::cout<<"\033[36;1m"<<"Chain_tip_link_: "	<< this->chain_tip_link 	<<"\033[36;0m"<<std::endl;
		std::cout<<"\033[36;1m"<<"Root frame: "			<< this->root_frame 		<<"\033[36;0m"<<std::endl;
		std::cout<<"\033[36;1m"<<"DOF: "				<< this->dof 				<<"\033[36;0m"<<std::endl;
		std::cout<<"\033[36;1m"<<"Nr_segments: "		<< this-> segments 			<<"\033[36;0m"<<std::endl;

		std::cout<<"\033[36;1m"<<"Joints name: "	<<"\033[36;0m"<<std::endl;
		for (uint16_t i = 0; i < this->kinematic_chain.getNrOfJoints(); ++i)
		{
			std::cout<<"\033[70;1m"	<< this->jnts.at(i).getName() <<"\033[70;0m"	<<std::endl;
		}



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

		std::cout<<"\033[36;1m"<<"Joints frames: "	<<"\033[36;0m"<<std::endl;
		for (std::vector<KDL::Frame>::const_iterator it = this->jnt_homo_mat.begin(); it!= jnt_homo_mat.end(); ++it)
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

/*
		std::vector<double> jnt_angels;
		jnt_angels.resize( 6, 0.0 );
		this->forwardKinematics(jnt_angels);
*/
}

void Kinematics::forwardKinematics(const std::vector<double>& jnt_angels)
{
	KDL::Frame fk_mat = KDL::Frame::Identity();

	for (uint16_t i = 0; i < this->segments; ++i)
	{

		fk_mat = fk_mat * this->jnt_homo_mat[i];


    	if (_DEBUG_)
    	{
			std::cout<<"\033[36;1m"<<"fk matrix of " << this->jnts.at(i).getName() <<"\033[36;0m"<<std::endl;
			KDL::Rotation rot_mat = fk_mat.M;
			KDL::Vector pos_mat = fk_mat.p;

				//for (unsigned int i = 0; i < it->)
				std::cout<<"\033[32;1m"	<<	" rxx "<< rot_mat(0,0) <<	" rxy "<< rot_mat(0,1) <<	" rxz "<< rot_mat(0,2)	<< "\n"
										<<	" ryx "<< rot_mat(1,0) <<	" ryy "<< rot_mat(1,1) <<	" ryz "<< rot_mat(1,2)	<< "\n"
										<<	" rzx "<< rot_mat(2,0) <<	" rzy "<< rot_mat(2,1) <<	" rzz "<< rot_mat(2,2)	<< "\n"
										<<	" px "<< pos_mat.x() <<	" py "<< pos_mat.y() <<	" pz "<< pos_mat.z()
						<<"\033[32;0m"<<std::endl;
    	}


	}
}

