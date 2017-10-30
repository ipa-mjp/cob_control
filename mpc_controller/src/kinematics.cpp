
#include <mpc_controller/kinematics.h>

using namespace nmpc;

Kinematics::Kinematics(const std::string rbt_description , const std::string& chain_base_link, const std::string& chain_tip_link, const std::string& root_frame)
{
	//base, tip link and root frame
	this->chain_base_link = chain_base_link;
	this->chain_tip_link = chain_tip_link;
	this->root_frame = root_frame;
	this->dof = 0;

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
    	std::cout<<rot.x()<<" , "<<rot.y()<<" , "<<rot.z()<<std::endl;

    	this->jnt_homo_mat.push_back(this->frames.at(i));

    	if (this->jnts.at(i).getType() == 0)	//revolute joint	//todo consider test for presmatic joint
    	{
    		this->createHomoRoatationMatrix( i );
    		this->dof++;						// consider revoulte joint as dof
    	}


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
    	this->printDataMemebers();
    	//std::cout<<"\033[20m"<<"###########  fk correctness ######### "	<<"\033[0m"<<std::endl;

    	KDL::JntArray jnt_angles = KDL::JntArray(this->dof);
    	jnt_angles(0) = 1.57;
    	jnt_angles(1) = 1.57;
    	jnt_angles(2) = 1.57;
    	jnt_angles(3) = 1.57;
		this->forwardKinematics(jnt_angles);

		this->kdl_forwardKinematics(jnt_angles);

		this->computeJacobian(jnt_angles);

		this->kdl_computeJacobian(jnt_angles);
    }
}

void Kinematics::createHomoRoatationMatrix(const uint16_t& seg_nr)
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

		this->jnt_homo_mat.at(seg_nr).M(0,0) = cos(angle);	this->jnt_homo_mat.at(seg_nr).M(0,1) = -1*sin(angle);	this->jnt_homo_mat.at(seg_nr).M(0,2) = 0;
		this->jnt_homo_mat.at(seg_nr).M(1,0) = sin(angle);	this->jnt_homo_mat.at(seg_nr).M(1,1) = cos(angle);	this->jnt_homo_mat.at(seg_nr).M(1,2) = 0;
		this->jnt_homo_mat.at(seg_nr).M(2,0) = 0;			this->jnt_homo_mat.at(seg_nr).M(2,1) = 0;			this->jnt_homo_mat.at(seg_nr).M(2,2) = 1;
	}
}

void Kinematics::createRoatationMatrix(const double& angle, const std::vector<unsigned int>& rot_axis, KDL::Frame& lcl_homo_mat)
{

	//x-axis rotation
	if ( (rot_axis[0] == 1 || rot_axis[0] == -1) && rot_axis[1] == 0 && rot_axis[2] == 0 )
	{

		if (_DEBUG_)
			std::cout<<"rot about x-axis with angel: "<< angle<<std::endl;

		lcl_homo_mat.M(0,0) = 1;	lcl_homo_mat.M(0,1) = 0;			lcl_homo_mat.M(0,2) = 0;
		lcl_homo_mat.M(1,0) = 0;	lcl_homo_mat.M(1,1) = cos(angle);	lcl_homo_mat.M(1,2) = -1*sin(angle);
		lcl_homo_mat.M(2,0) = 0;	lcl_homo_mat.M(2,1) = sin(angle);	lcl_homo_mat.M(2,2) = cos(angle);

	}

	if ( rot_axis[0] == 0 && (rot_axis[1] == 1 || rot_axis[1] == -1) && rot_axis[2] == 0 )
	{

		if (_DEBUG_)
			std::cout<<"rot about y-axis with angel: "<< angle<<std::endl;

		lcl_homo_mat.M(0,0) = cos(angle);		lcl_homo_mat.M(0,1) = 0;			lcl_homo_mat.M(0,2) = sin(angle);
		lcl_homo_mat.M(1,0) = 0;				lcl_homo_mat.M(1,1) = 1;			lcl_homo_mat.M(1,2) = 0;
		lcl_homo_mat.M(2,0) = -1*sin(angle);	lcl_homo_mat.M(2,1) = 0;			lcl_homo_mat.M(2,2) = cos(angle);
	}

	if ( rot_axis[0] == 0 && rot_axis[1] == 0 && (rot_axis[2] == 1 || rot_axis[2] == -1) )
	{
		if (_DEBUG_)
			std::cout<<"rot about z-axis with angel: "<< angle<<std::endl;

		lcl_homo_mat.M(0,0) = cos(angle);	lcl_homo_mat.M(0,1) = -1*sin(angle);	lcl_homo_mat.M(0,2) = 0;
		lcl_homo_mat.M(1,0) = sin(angle);	lcl_homo_mat.M(1,1) = cos(angle);	lcl_homo_mat.M(1,2) = 0;
		lcl_homo_mat.M(2,0) = 0;			lcl_homo_mat.M(2,1) = 0;			lcl_homo_mat.M(2,2) = 1;
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

//todo: here hard-coded rot_axis, set proper way
void Kinematics::forwardKinematics(const KDL::JntArray& jnt_angels)
{
	KDL::Frame fk_mat = KDL::Frame::Identity();
	std::vector<unsigned int> rot_axis{0,0,1};
	unsigned int cnt = 0;

	//find transformation between chain_base_link & root frame if different
	if (this->root_frame != this->chain_base_link)
	{
		tf::TransformListener listener;
		tf::StampedTransform transform;
		try
		{	//world arm_1_link
			listener.waitForTransform(this->root_frame,this->chain_base_link, ros::Time(0), ros::Duration(5.0));	//link2,3 = 3.0,link4,5 = 4.0, link6,7 = 5.0
			listener.lookupTransform(this->root_frame, this->chain_base_link, ros::Time(0), transform);

			geometry_msgs::TransformStamped msg;
			tf::transformStampedTFToMsg(transform,  msg);
			fk_mat = tf2::transformToKDL(msg);

		}
		catch (tf::TransformException ex)
		{
		      ROS_ERROR("%s",ex.what());
		      ros::Duration(1.0).sleep();
		 }

	}

	//comput fk from chain base_link to chain tip link, think with dof
	for (uint16_t i = 0; i < this->segments; ++i)
	{
		//if revolute joint than multiply with joint angles
		if (this->jnts.at(i).getType() == 0)
		{

			KDL::Frame lcl_homo_mat = KDL::Frame::Identity();
			this->createRoatationMatrix( jnt_angels(cnt), rot_axis, lcl_homo_mat );
			this->jnt_homo_mat[i] =	this->jnt_homo_mat[i] * lcl_homo_mat;
			cnt++;

			/*
	    	if (_DEBUG_)
	    	{
				std::cout<<"\033[36;1m"<<"lcl homo matrix of " << this->jnts.at(i).getName() <<"\033[36;0m"<<std::endl;
				KDL::Rotation rot_mat = lcl_homo_mat.M;
				KDL::Vector pos_mat = lcl_homo_mat.p;

					//for (unsigned int i = 0; i < it->)
					std::cout<<"\033[32;1m"	<<	" rxx "<< rot_mat(0,0) <<	" rxy "<< rot_mat(0,1) <<	" rxz "<< rot_mat(0,2)	<< "\n"
																<<	" ryx "<< rot_mat(1,0) <<	" ryy "<< rot_mat(1,1) <<	" ryz "<< rot_mat(1,2)	<< "\n"
																<<	" rzx "<< rot_mat(2,0) <<	" rzy "<< rot_mat(2,1) <<	" rzz "<< rot_mat(2,2)	<< "\n"
																<<	" px "<< pos_mat.x() <<	" py "<< pos_mat.y() <<	" pz "<< pos_mat.z()
							<<"\033[32;0m"<<std::endl;
	    	}



	    	if (_DEBUG_)
	    	{
				std::cout<<"\033[36;1m"<<"New homo matrix of " << this->jnts.at(i).getName() <<"\033[36;0m"<<std::endl;
				KDL::Rotation rot_mat = this->jnt_homo_mat.at(i).M;
				KDL::Vector pos_mat = this->jnt_homo_mat.at(i).p;

					//for (unsigned int i = 0; i < it->)
					std::cout<<"\033[32;1m"	<<	" rxx "<< rot_mat(0,0) <<	" rxy "<< rot_mat(0,1) <<	" rxz "<< rot_mat(0,2)	<< "\n"
																<<	" ryx "<< rot_mat(1,0) <<	" ryy "<< rot_mat(1,1) <<	" ryz "<< rot_mat(1,2)	<< "\n"
																<<	" rzx "<< rot_mat(2,0) <<	" rzy "<< rot_mat(2,1) <<	" rzz "<< rot_mat(2,2)	<< "\n"
																<<	" px "<< pos_mat.x() <<	" py "<< pos_mat.y() <<	" pz "<< pos_mat.z()
							<<"\033[32;0m"<<std::endl;
	    	}*/



		}

		fk_mat = fk_mat * this->jnt_homo_mat[i];
		jnt_fk_mat.push_back(fk_mat);

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

	this->fk_mat = fk_mat;

}

//todo: can not find fk from root frame
void Kinematics::kdl_forwardKinematics(const KDL::JntArray& jnt_angels)
{
	using namespace KDL;
	KDL::Frame fk_mat = KDL::Frame::Identity();
	ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(this->kinematic_chain);


	//find transformation between chain_base_link & root frame if different
		if (this->root_frame != this->chain_base_link)
		{
			tf::TransformListener listener;
			tf::StampedTransform transform;
			try
			{	//world arm_1_link
				listener.waitForTransform(this->root_frame,this->chain_base_link, ros::Time(0), ros::Duration(5.0));	//link2,3 = 3.0,link4,5 = 4.0, link6,7 = 5.0
				listener.lookupTransform(this->root_frame, this->chain_base_link, ros::Time(0), transform);

				geometry_msgs::TransformStamped msg;
				tf::transformStampedTFToMsg(transform,  msg);
				fk_mat = tf2::transformToKDL(msg);

			}
			catch (tf::TransformException ex)
			{
			      ROS_ERROR("%s",ex.what());
			      ros::Duration(1.0).sleep();
			 }

		}

	bool kinematic_status = fksolver.JntToCart(jnt_angels, fk_mat);

	if (_DEBUG_)
	{
			std::cout<<"\033[36;1m"<<"kdl fk matrix of " <<"\033[36;0m"<<std::endl;
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


//todo: make jacobian function independance of fk by directly using info of jnt_angles
void Kinematics::computeJacobian(const KDL::JntArray& jnt_angels)
{
	//todo: change dimension of matrix (means 7) accord to dof

	typedef Eigen::Matrix<double, 3, 1>       Cart3Vector;

	 Cart3Vector p(0,0,0);	Cart3Vector z_0(0,0,1); 	Cart3Vector p_0(0,0,0);

	// dist from end-effector to base-link
	p(0) = fk_mat.p.x();	p(1) = fk_mat.p.y();	p(2) = fk_mat.p.z();

	if (_DEBUG_)
		{
				std::cout<<"\033[94m"<<"End-effector pose vector relative to base link " <<"\033[0m"<<std::endl;
				std::cout<<"\033[94m" << p  <<"\033[0m" <<std::endl;
		}


	//compute linear and angular velocity at each joint
	for(uint16_t i = 0; i < this->segments; ++i)
	{
		Cart3Vector J_v(0,0,0), J_o(0,0,0);
		if (i == 0)
		{
			if ( this-> jnts.at(i).getType() == 0 )	//revolute joint
			{
				J_v = z_0.cross( p );
				J_o = z_0;
			}

			if ( this-> jnts.at(i).getType() == 8 )	//prismatic joint
			{
				J_v = z_0;
				J_o = p_0;
			}
			else
			{
				;
			}
		}
		else
		{
			Cart3Vector z_i(0,0,0);	Cart3Vector p_i(0,0,0);

			//third col of rot matrix at each jnt with respect to base link
			z_i(0) = jnt_fk_mat.at(i).M(0,2);		z_i(1) = jnt_fk_mat.at(i).M(1,2);		z_i(2) = jnt_fk_mat.at(i).M(2,2);
			p_i(0) = jnt_fk_mat.at(i).p(0);			p_i(1) = jnt_fk_mat.at(i).p(1);		p_i(2) = jnt_fk_mat.at(i).p(2);

			if ( this-> jnts.at(i).getType() == 0 )	//revolute joint
			{
				J_v = z_i.cross( p - p_i );
				J_o = z_i;
			}

			if ( this-> jnts.at(i).getType() == 8 )	//prismatic joint
			{
				J_v = z_i;
				J_o = p_0;
			}
			else
			{
				;
			}
		}

		JacobianMatrix(0,i) = J_v(0);	JacobianMatrix(1,i) = J_v(1);	JacobianMatrix(2,i) = J_v(2);
		JacobianMatrix(3,i) = J_o(0);	JacobianMatrix(4,i) = J_o(1);	JacobianMatrix(5,i) = J_o(2);
	}

	if (_DEBUG_)
	{
			std::cout<<"\033[20m"<<"Jacobian Matrix " <<"\033[0m"<<std::endl;

			std::cout<<"\033[20m"	<< JacobianMatrix << "\t" <<"\033[0m"<<std::endl;

	}


}

void Kinematics::kdl_computeJacobian(const KDL::JntArray& jnt_angels)
{

	KDL::ChainJntToJacSolver jacobi_solver = KDL::ChainJntToJacSolver(this->kinematic_chain);

		KDL::Jacobian j_kdl = KDL::Jacobian(this->dof);
		int jacobian_state = jacobi_solver.JntToJac(jnt_angels, j_kdl);

		Eigen::Matrix<double, 6, 7>  JacobianMatrix;

		for (unsigned int i = 0; i < 6; ++i)
				{
					for (unsigned int j = 0; j < this->dof; ++j)
					{
						JacobianMatrix(i,j) = j_kdl(i,j);
					}
				}

		if (_DEBUG_)
		{
				std::cout<<"\033[94m"<<"kdl Jacobian Matrix " <<"\033[0m"<<std::endl;

				std::cout<<"\033[94m"	<< JacobianMatrix << "\t" <<"\033[0m"<<std::endl;

		}


}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
inline
std::string Kinematics::getChainBaseLink(void)
{
	return this->chain_base_link;
}

inline
void Kinematics::getChainBaseLink(std::string& base_link)
{
	base_link = this->chain_base_link;
}

inline
std::string Kinematics::getChainTipLink(void)
{
	return this->chain_tip_link;
}

inline
void Kinematics::getChainTipLink(std::string& tip_link)
{
	tip_link = this->chain_tip_link;
}

inline
std::string Kinematics::getChainRootLink(void)
{
	return this->root_frame;
}

inline
void Kinematics::getChainRootLink(std::string& root_frame)
{
	root_frame = this->root_frame;
}

inline
uint16_t Kinematics::getNumberOfJnts(void)
{
	return this->dof;
}

inline
void Kinematics::getNumberOfJnts(uint16_t& nr_jnts)
{
	nr_jnts = this->dof;
}

uint16_t Kinematics::getNumberOfSegments(void)
{
	return this->segments;
}
void getNumberOfSegments(uint16_t& nr_segments);
