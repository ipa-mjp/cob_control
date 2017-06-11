/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2017 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: care-o-bot
 * \note
 *   ROS stack name: cob_control
 * \note
 *   ROS package name: cob_nonlinear_mpc
 *
 * \author
 *   Author: Bruno Brito  email: Bruno.Brito@ipa.fraunhofer.de
 *   Christoph Mark, email: Christoph.Mark@ipa.fraunhofer.de
 *
 * \date Date of creation: April, 2017
 *
 * \brief
 *
 *
 ****************************************************************/

#include <cob_nonlinear_mpc/nonlinear_mpc.h>

void MPC::init(){
    // Total number of NLP variables
    NV = state_dim_*(num_shooting_nodes_+1) +control_dim_*num_shooting_nodes_;

    V = MX::sym("V",NV);
    vector<double> tmp;
    for(int k=0; k < num_shooting_nodes_; ++k)
    {
        tmp.clear();
        for(int i=0; i < control_dim_; ++i)
        {
            tmp.push_back(0);
        }
        u_open_loop_.push_back(tmp);
    }

    for(int k=1; k <= num_shooting_nodes_; ++k)
    {
        tmp.clear();

        for(int i=0; i < state_dim_; ++i)
        {
            tmp.push_back(0);
        }

        x_open_loop_.push_back(tmp);
    }

    for(int i = 0; i < control_dim_; i++)
    {
        ROS_INFO_STREAM("CONTROL DIM: "<<control_dim_);
        u_init_.push_back(0);
    }
}

int MPC::get_num_shooting_nodes(){
    return this->num_shooting_nodes_;
}

double MPC::get_time_horizon(){
    return this->time_horizon_;
}

int MPC::get_state_dim(){
    return this->state_dim_;
}

int MPC::get_control_dim(){
    return this->control_dim_;
}

void MPC::set_path_constraints(vector<double> state_path_constraints_min,vector<double> state_path_constraints_max){
    this->state_path_constraints_min_=state_path_constraints_min;
    this->state_path_constraints_max_=state_path_constraints_max;
}

void MPC::set_state_constraints(vector<double> state_terminal_constraints_min,vector<double> state_terminal_constraints_max){
    this->state_terminal_constraints_min_=state_terminal_constraints_min;
    this->state_terminal_constraints_max_=state_terminal_constraints_max;
}

void MPC::set_input_constraints(vector<double> input_constraints_min,vector<double> input_constraints_max){
    this->input_constraints_min_=input_constraints_min;
    this->input_constraints_max_=input_constraints_max;
}

void MPC::generate_symbolic_forward_kinematics(Robot* robot){
    // Casadi symbolics
    ROS_INFO("MPC::generate_symbolic_forward_kinematics");
    u_ = SX::sym("u", control_dim_);  // control
    x_ = SX::sym("x", state_dim_); // states

    SX T = SX::sym("T",4,4);
    if(robot->base_active_){
    ////generic rotation matrix around z and translation vector for x and y
        T(0,0) = cos(x_(2)); T(0,1) = -sin(x_(2));  T(0,2) = 0.0; T(0,3) = x_(0);
        T(1,0) = sin(x_(2)); T(1,1) = cos(x_(2));   T(1,2) = 0.0; T(1,3) = x_(1);
        T(2,0) = 0.0;        T(2,1) = 0.0;          T(2,2) = 1.0; T(2,3) = 0;
        T(3,0) = 0.0;        T(3,1) = 0.0;          T(3,2) = 0.0; T(3,3) = 1.0;
        fk_base_ = T; //
        ROS_INFO("Base forward kinematics");
    }
    int offset;

    for(int i=0;i<robot->joint_frames.size();i++){

        KDL::Vector pos;
        KDL::Rotation rot;
        rot=robot->joint_frames.at(i).M;
        pos=robot->joint_frames.at(i).p;
#ifdef __DEBUG__
        ROS_WARN("Rotation matrix %f %f %f \n %f %f %f \n %f %f %f \n",rot(0,0),rot(0,1),rot(0,2),rot(1,0),rot(1,1),rot(1,2),rot(2,0),rot(2,1),rot(2,2));
        ROS_INFO_STREAM("Joint position of transformation"<< " X: " << pos.x()<< " Y: " << pos.y()<< " Z: " << pos.z());
#endif
        if(robot->base_active_){ // if base active first initial control variable belong to the base
        // here each joint is considered to be revolute.. code needs to be updated for prismatic
        //rotation matrix of the joint * homogenic transformation matrix of the next joint relative to the previous
        //still needs to be improved... if the chain is composed by joint than not joint than joint again this is going to be wrong
            T(0,0) = rot(0,0)*cos(x_(i+3))+rot(0,1)*sin(x_(i+3));
            T(0,1) = -rot(0,0)*sin(x_(i+3))+rot(0,1)*cos(x_(i+3));
            T(0,2) = rot(0,2); T(0,3) = pos.x();
            T(1,0) = rot(1,0)*cos(x_(i+3))+rot(1,1)*sin(x_(i+3));
            T(1,1) = -rot(1,0)*sin(x_(i+3))+rot(1,1)*cos(x_(i+3));
            T(1,2) = rot(1,2); T(1,3) = pos.y();
            T(2,0) = rot(2,0)*cos(x_(i+3))+rot(2,1)*sin(x_(i+3));
            T(2,1) = -rot(2,0)*sin(x_(i+3))+rot(2,1)*cos(x_(i+3));
            T(2,2) = rot(2,2); T(2,3) = pos.z();
            T(3,0) = 0.0; T(3,1) = 0.0; T(3,2) = 0.0; T(3,3) = 1.0;
        }
        else{
            T(0,0) = rot(0,0)*cos(x_(i))+rot(0,1)*sin(x_(i));
            T(0,1) = -rot(0,0)*sin(x_(i))+rot(0,1)*cos(x_(i));
            T(0,2) = rot(0,2); T(0,3) = pos.x();
            T(1,0) = rot(1,0)*cos(x_(i))+rot(1,1)*sin(x_(i));
            T(1,1) = -rot(1,0)*sin(x_(i))+rot(1,1)*cos(x_(i));
            T(1,2) = rot(1,2); T(1,3) = pos.y();
            T(2,0) = rot(2,0)*cos(x_(i))+rot(2,1)*sin(x_(i));
            T(2,1) = -rot(2,0)*sin(x_(i))+rot(2,1)*cos(x_(i));
            T(2,2) = rot(2,2); T(2,3) = pos.z();
            T(3,0) = 0.0; T(3,1) = 0.0; T(3,2) = 0.0; T(3,3) = 1.0;
        }

        T_BVH p;
        p.link = robot->kinematic_chain.getSegment(i).getName();
        p.T = T;
        this->BV.transform_vec_bvh_.push_back(p);
    }

    // Get Endeffector FK
    for(int i=0; i< this->BV.transform_vec_bvh_.size(); i++)
    {
        if(robot->base_active_)
        {
            if(i==0)
            {
                ROS_WARN("BASE IS ACTIVE");
                fk_ = mtimes(fk_base_,this->BV.transform_vec_bvh_.at(i).T);
            }
            else
            {
                fk_ = mtimes(fk_,this->BV.transform_vec_bvh_.at(i).T);
            }
        }
        else
        {
            if(i==0)
            {
                fk_ = this->BV.transform_vec_bvh_.at(i).T;
            }
            else
            {
                fk_ = mtimes(fk_,this->BV.transform_vec_bvh_.at(i).T);
            }
        }
        fk_vector_.push_back(fk_); // stacks up multiplied transformation until link n
    }

    u_min =  this->input_constraints_min_;
    u_max  = this->input_constraints_max_;

    ROS_INFO("Bounds and initial guess for the state");

    x_min  = this->state_path_constraints_min_;
    x_max  = this->state_path_constraints_max_;
    xf_min = this->state_terminal_constraints_min_;
    xf_max = this->state_terminal_constraints_max_;

    weiting.resize(control_dim_,1);
#ifdef __DEBUG__
    ROS_INFO("ROBOT MASSES");
    for(int i=0;i<robot->forward_kinematics.size();i++){
        ROS_INFO_STREAM("Segment " <<robot->forward_kinematics.at(i).getName());
        ROS_INFO_STREAM("\n Segment x: " <<robot->forward_kinematics.at(i).getFrameToTip().p.x());
        ROS_INFO_STREAM(" y:" <<robot->forward_kinematics.at(i).getFrameToTip().p.y());
        ROS_INFO_STREAM(" z:" <<robot->forward_kinematics.at(i).getFrameToTip().p.z());
        ROS_INFO_STREAM(" Mass:" <<robot->kinematic_chain.getSegment(i).getInertia().getMass());
    }
#endif
    if(robot->base_active_){
        for(int i=0;i<3;i++){
            weiting.at(i)=robot->kinematic_chain.getSegment(0).getInertia().getMass();
        }
        vector<double> masses;
        for(int i=0;i<robot->forward_kinematics.size();i++){
            if(robot->kinematic_chain.getSegment(i).getJoint().getType()==0){
                masses.push_back(robot->kinematic_chain.getSegment(i).getInertia().getMass());
                ROS_INFO("JOINT MASSES: %f", robot->kinematic_chain.getSegment(i).getInertia().getMass());
            }
        }

        for(int i=3;i<control_dim_;i++){
            weiting.at(i)=masses.at(i-3);
        }
    }
    else{
        vector<double> masses;
        for(int i=0;i<robot->forward_kinematics.size();i++){
            if(robot->kinematic_chain.getSegment(i).getJoint().getType()==0){
                masses.push_back(robot->kinematic_chain.getSegment(i).getInertia().getMass());
                ROS_INFO("JOINT MASSES: %f", robot->kinematic_chain.getSegment(i).getInertia().getMass());
            }
        }

        for(int i=0;i<control_dim_;i++){
            weiting.at(i)=masses.at(i);
        }
    }
    R = SX::sym("R",control_dim_,control_dim_);

}


Eigen::MatrixXd MPC::mpc_step(const geometry_msgs::Pose pose, const KDL::JntArray& state,Robot* robot)
{

    // Bounds and initial guess for the control
#ifdef __DEBUG__
    ROS_INFO_STREAM("input_constraints_min_: " <<this->input_constraints_min_.size());
    ROS_INFO_STREAM("input_constraints_max_: " <<this->input_constraints_max_.size());
#endif

    //ROS_INFO("ODE right hand side and quadrature");
    SX qdot = SX::vertcat({u_});

    //ROS_INFO("Current Quaternion and Position Vector.");

    q_c = SX::vertcat({ //current quaternion
        0.5 * sqrt(fk_(0,0) + fk_(1,1) + fk_(2,2) + 1.0),
        0.5 * ((fk_(2,1) - fk_(1,2))) / sqrt(fk_(0,0) + fk_(1,1) + fk_(2,2) + 1.0),
        0.5 * ((fk_(0,2) - fk_(2,0))) / sqrt(fk_(1,1) + fk_(2,2) + fk_(0,0) + 1.0),
        0.5 * ((fk_(1,0) - fk_(0,1))) / sqrt(fk_(2,2) + fk_(0,0) + fk_(1,1) + 1.0)
    });

    pos_c = SX::vertcat({fk_(0,3), fk_(1,3), fk_(2,3)}); //current state
    //ROS_INFO("Desired Goal-pose");
    pos_target = SX::vertcat({pose.position.x, pose.position.y, pose.position.z});
    q_target = SX::vertcat({pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z});
    q_target.print(std::cout);
    // Prevent collision with Base_link
    SX barrier;
    SX dist;
/*
    std::unordered_map<std::string, std::vector<std::string> >::iterator it_scm;

    int counter = 0;
    double bv_radius;

    for( it_scm = self_collision_map_.begin(); it_scm != self_collision_map_.end(); it_scm++)
    {
        std::vector<string> scm_collision_links = it_scm->second;
        for(int i=0; i<scm_collision_links.size(); i++)
        {
            ROS_WARN_STREAM(it_scm->first);
            vector<vector<SX>> p1_mat = bvh_matrix[it_scm->first];
            vector<vector<SX>> p2_mat = bvh_matrix[scm_collision_links.at(i)];

            for(int k=0; k<p1_mat.size(); k++)
            {
                if(it_scm->first == "body")
                {
                    bv_radius = bvb_radius_.at(k);
                }
                else
                {
                    bv_radius = 0.1;
                }

                vector<SX> p1_vec = p1_mat.at(k);
                for(int m=0; m<p2_mat.size(); m++)
                {
                    vector<SX> p2_vec = p2_mat.at(m);

                    SX p1 = SX::vertcat({p1_vec.at(0)});
                    SX p2 = SX::vertcat({p2_vec.at(0)});
                    dist = dot(p1 - p2, p1 - p2);

                    if(counter == 0)
                    {
                        barrier = exp((bv_radius - sqrt(dist))/0.01);
                        counter = 1;
                    }
                    else
                    {
                        barrier += exp((bv_radius - sqrt(dist))/0.01);
                    }
                }
            }
        }
    }*/

    // Get orientation error
    SX q_c_inverse = SX::vertcat({q_c(0), -q_c(1), -q_c(2), -q_c(3)});
    SX e_quat= quaternion_product(q_c_inverse,q_target);
    SX error_attitute = SX::vertcat({ e_quat(1), e_quat(2), e_quat(3)});
    SX q = SX::vertcat({e_quat(0), e_quat(1), e_quat(2),e_quat(3)});
    Function qk = Function("e_quat", {x_}, {q});
    vector<double> x;
    for(unsigned int i=0; i < state.rows();i++)
    {
        x.push_back(state(i));
    }
    SX test_v = qk(SX::vertcat({x})).at(0);

    //ROS_INFO("L2 norm of the control signal");
    ROS_INFO_STREAM("ATTITUDE ERROR: " << (double)test_v(0) <<" " << (double)test_v(1) <<" "<< (double)test_v(2) <<" "<< (double)test_v(3));
    this->acceleration_coordination(state);
    //R.print(std::cout);
    SX u2 = SX::mtimes(R,u_);
    //u2.print(std::cout);
    SX energy = SX::dot(u_,u2);
    //energy.print(std::cout);
    //ROS_INFO("L2 norm of the states");
    std::vector<int> state_convariance(state_dim_,1);
    SX S = 0.01*SX(state_convariance);
    SX motion = dot(sqrt(S)*x_,sqrt(S)*x_);

    //ROS_INFO_STREAM("STATE COVARIANCE: "<< S);
    //ROS_INFO("Objective");
    SX error=pos_c-pos_target;

    SX L = 10 * dot(error_attitute,error_attitute) + dot(pos_c-pos_target,pos_c-pos_target) ;//+ barrier;

    //ROS_INFO("Create Euler integrator function");
    Function F = create_integrator(state_dim_, control_dim_, time_horizon_, num_shooting_nodes_, qdot, x_, u_, L);

    // Offset in V
    int offset=this->init_shooting_node();

    //ROS_INFO("(Make sure that the size of the variable vector is consistent with the number of variables that we have referenced");
    casadi_assert(offset==NV);

    // Objective function
    MX J = 0;

    //Constraint function and bounds
    vector<MX> g;

    //ROS_INFO("Loop over shooting nodes");
    for(unsigned int k=0; k<num_shooting_nodes_; ++k)
    {
        // Create an evaluation node
        MXDict I_out = F( MXDict{ {"x0", X[k]}, {"p", U[k]} });

        // Save continuity constraints
        g.push_back( I_out.at("xf") - X[k+1] );

        // Add objective function contribution
        J += I_out.at("qf");
    }

    ROS_INFO("NLP");
    MXDict nlp = {{"x", V}, {"f", J}, {"g", vertcat(g)}};

    // Set options
    Dict opts;

    opts["ipopt.tol"] = 1e-4;
    opts["ipopt.max_iter"] = 20;
//    opts["ipopt.hessian_approximation"] = "limited-memory";
//    opts["ipopt.hessian_constant"] = "yes";
    opts["ipopt.linear_solver"] = "ma27";
    opts["ipopt.print_level"] = 0;
    opts["print_time"] = true;
    opts["expand"] = true;  // Removes overhead

    ROS_INFO("Create an NLP solver and buffers");
    Function solver = nlpsol("nlpsol", "ipopt", nlp, opts);

    std::map<std::string, DM> arg, res;

    //ROS_INFO("Bounds and initial guess");
    arg["lbx"] = min_state;
    arg["ubx"] = max_state;
    arg["lbg"] = 0;
    arg["ubg"] = 0;
    arg["x0"] = init_state;
    /*ROS_INFO_STREAM("INIT STATE DIM:" << init_state.size());
    ROS_INFO_STREAM("min INIT STATE DIM:" << min_state.size());
    ROS_INFO_STREAM("max STATE DIM:" << max_state.size());*/
    ROS_INFO("Solve the problem");
    res = solver(arg);

    // Optimal solution of the NLP
    vector<double> V_opt(res.at("x"));
    vector<double> J_opt(res.at("f"));

    ROS_INFO("Get the optimal control");
    Eigen::VectorXd q_dot = Eigen::VectorXd::Zero(state_dim_);
//    Eigen::VectorXd x_new = Eigen::VectorXd::Zero(mpc_ctr_->get_state_dim());
    SX sx_x_new;
    u_open_loop_.clear();
    x_open_loop_.clear();
    x_new.clear();

    for(int i=0; i<1; ++i)  // Copy only the first optimal control sequence
    {
        for(int j=0; j<control_dim_; ++j)
        {
            q_dot[j] = V_opt.at(state_dim_ + j);
            x_new.push_back(V_opt.at(j));
        }
    }
    sx_x_new = SX::vertcat({x_new});

    // Safe optimal control sequence at time t_k and take it as inital guess at t_k+1

    ROS_INFO("Plot bounding volumes");
    geometry_msgs::Point point;
    point.x = 0;
    point.y = 0;
    point.z = 0;

    /*SX result;
    for( it_scm = self_collision_map_.begin(); it_scm != self_collision_map_.end(); it_scm++)
    {
        vector<string> tmp = it_scm->second;

        for(int i=0; i<tmp.size();i++)
        {
            vector<vector<SX>> SX_vec = bvh_matrix[tmp.at(i)];
            for(int k=0; k<SX_vec.size(); k++)
            {
                SX test = SX::horzcat({SX_vec.at(k).at(0)});
                Function tesssst = Function("test", {x_}, {test});
                result = tesssst(sx_x_new).at(0);
                point.x = (double)result(0);
                point.y = (double)result(1);
                point.z = (double)result(2);

                bv_radius = 0.1;
                visualizeBVH(point, bv_radius, i+i*tmp.size());
            }
        }


        vector<vector<SX>> SX_vec = bvh_matrix[it_scm->first];
        for(int k=0; k<SX_vec.size(); k++)
        {
            SX test = SX::horzcat({SX_vec.at(k).at(0)});
            Function tesssst = Function("test", {x_}, {test});
            result = tesssst(sx_x_new).at(0);
            point.x = (double)result(0);
            point.y = (double)result(1);
            point.z = (double)result(2);

            if(it_scm->first == "body")
            {
                bv_radius = bvb_radius_.at(k);
            }
            else
            {
                bv_radius = 0.1;
            }
            visualizeBVH(point, bv_radius, k+tmp.size()+SX_vec.size());
        }
    }*/

    KDL::Frame ef_pos = forward_kinematics(state);

    return q_dot;
}
int MPC::init_shooting_node(){
    // Offset in V
    int offset=0;
    X.clear();
    U.clear();
    min_state.clear();
    max_state.clear();
    init_state.clear();

    //THIS CAN BE SERIOULSY OPTIMISED
    for(unsigned int k=0; k<num_shooting_nodes_; ++k)
    {
        //ROS_INFO("Local state");
        X.push_back( V.nz(Slice(offset,offset+state_dim_)));

        if(k==0)
        {
            min_state.insert(min_state.end(), x0_min.begin(), x0_min.end());
            max_state.insert(max_state.end(), x0_max.begin(), x0_max.end());
        }
        else
        {
            min_state.insert(min_state.end(), x_min.begin(), x_min.end());
            max_state.insert(max_state.end(), x_max.begin(), x_max.end());
        }
        init_state.insert(init_state.end(), x_init.begin(), x_init.end());
        offset += state_dim_;

        //ROS_INFO("Local control via shift initialization");
        U.push_back( V.nz(Slice(offset,offset+control_dim_)));
        min_state.insert(min_state.end(), u_min.begin(), u_min.end());
        max_state.insert(max_state.end(), u_max.begin(), u_max.end());

        init_state.insert(init_state.end(), u_init_.begin(), u_init_.end());
        offset += control_dim_;
    }

    //ROS_INFO("State at end");
    X.push_back(V.nz(Slice(offset,offset+state_dim_)));
    min_state.insert(min_state.end(), xf_min.begin(), xf_min.end());
    max_state.insert(max_state.end(), xf_max.begin(), xf_max.end());
    init_state.insert(init_state.end(), u_init_.begin(), u_init_.end());

    x0_min.clear();
    x0_max.clear();
    x_init.clear();
    return offset += state_dim_;
}
void MPC::acceleration_coordination(const KDL::JntArray& state){
    for(int i=0; i<weiting.size();i++){
        for(int j=0; j<weiting.size();j++){
            R(i,j)=0.0;
        }
        R(i,i)=weiting.at(i);
    }
}

KDL::Frame MPC::forward_kinematics(const KDL::JntArray& state){

    KDL::Frame ef_pos; //POsition of the end effector

    Function fk = Function("fk_", {x_}, {pos_c});

    vector<double> x;
    for(unsigned int i=0; i < state.rows();i++)
    {
        x.push_back(state(i));
    }
    SX test_v = fk(SX::vertcat({x})).at(0);
    //SX test_base = fk_base(SX::vertcat({x_new})).at(0);
    ef_pos.p.x((double)test_v(0));
    ef_pos.p.y((double)test_v(1));
    ef_pos.p.z((double)test_v(2));
    ROS_INFO_STREAM("Joint values:" << x);
    ROS_WARN_STREAM("Current Position: \n" << ef_pos.p.x() << " "<< ef_pos.p.y() << " "<< ef_pos.p.z() << " ");
    //ROS_WARN_STREAM("Base Position: \n" << (double)test_base(0) << " "<< (double)test_base(1) << " "<< (double)test_base(2) << " ");
    ROS_WARN_STREAM("Target Position: \n" << pos_target);
    return ef_pos;
}

Function MPC::create_integrator(const unsigned int state_dim, const unsigned int control_dim, const double T,
                                            const unsigned int N, SX ode, SX x, SX u, SX L)
{
    // Euler discretize
    double dt = T/((double)N);

    Function f = Function("f", {x, u}, {ode, L});

    MX X0 = MX::sym("X0", state_dim);
    MX U_ = MX::sym("U",control_dim);
    MX X_ = X0;
    MX Q = 0;

    vector<MX> input(2);
    input[0] = X_;
    input[1] = U_;
    MX qdot_new = f(input).at(0);
    MX Q_new = f(input).at(1);

    X_= X_+ dt * qdot_new;
    Q = Q + dt * Q_new;

    Function F = Function("F", {X0, U_}, {X_, Q}, {"x0","p"}, {"xf", "qf"});
    return F.expand("F");   // Remove overhead
}

SX MPC::dual_quaternion_product(SX q1, SX q2)
{
    SX q1_real = SX::vertcat({q1(0),q1(1),q1(2),q1(3)});
    SX q1_dual = SX::vertcat({q1(4),q1(5),q1(6),q1(7)});
    SX q2_real = SX::vertcat({q2(0),q2(1),q2(2),q2(3)});
    SX q2_dual = SX::vertcat({q2(4),q2(5),q2(6),q2(7)});

    SX q1q2_real = quaternion_product(q1_real,q2_real);
    SX q1_real_q2_dual = quaternion_product(q1_real,q2_dual);
    SX q1_dual_q2_real = quaternion_product(q1_dual,q2_real);

    SX q_prod = SX::vertcat({
        q1q2_real,
        q1_real_q2_dual + q1_dual_q2_real
    });

    return q_prod;
}

SX MPC::quaternion_product(SX q1, SX q2)
{
    SX q1_v = SX::vertcat({q1(1),q1(2),q1(3)});
    SX q2_v = SX::vertcat({q2(1),q2(2),q2(3)});

    SX c = SX::cross(q1_v,q2_v);

    SX q_new = SX::vertcat({
        q1(0) * q2(0) - dot(q1_v,q2_v),
        q1(0) * q2(1) + q2(0) * q1(1) + c(0),
        q1(0) * q2(2) + q2(0) * q1(2) + c(1),
        q1(0) * q2(3) + q2(0) * q1(3) + c(2)
    });

    return q_new;
}
