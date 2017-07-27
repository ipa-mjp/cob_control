#!/usr/bin/env python
"""
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
 *   ROS package name: cob_nmpc_controller
 *
 * \author
 *   Author: Bruno Brito, email: Bruno.Brito@ipa.fraunhofer.de
 *
 * \date Date of creation: July, 2017
 *
 * \brief
 *
 *
"""

import numpy as np

import PyKDL as kdl
import rospy

from sensor_msgs.msg import JointState
from kdl_parser_py.urdf import treeFromUrdfModel
from urdf_parser_py.urdf import Robot
import tf

from casadi import *

pi = 3.1415

class Kinematics(object):
    ##
    # Constructor
    # @param urdf URDF object of robot.
    # @param base_link Name of the root link of the kinematic chain.
    # @param end_link Name of the end link of the kinematic chain.
    # @param kdl_tree Optional KDL.Tree object to use. If None, one will be generated
    #                          from the URDF.
    def __init__(self, urdf, base_link, end_link, kdl_tree=None):
        if kdl_tree is None:
            [tree_ok, kdl_tree] = treeFromUrdfModel(urdf)

        if not tree_ok:
            rospy.logerr("KDLKinematics URDF tree parsing returned with error.")
            raise Exception("KDLKinematics URDF tree parsing returned with error.")

        self.tree = kdl_tree
        self.urdf = urdf


        base_link = base_link.split("/")[-1] # for dealing with tf convention
        end_link = end_link.split("/")[-1] # for dealing with tf convention
        self.chain = kdl_tree.getChain(base_link, end_link)

        self.base_link = base_link
        self.end_link = end_link

        # record joint information in easy-to-use lists
        self.joint_limits_lower = []
        self.joint_limits_upper = []
        self.joint_safety_lower = []
        self.joint_safety_upper = []
        self.joint_types = []
        self.joint_states = []
        self.tf_list = []
        print base_link
        print end_link
        for jnt_name in self.get_joint_names():
            jnt = urdf.joint_map[jnt_name]

            if jnt.limit is not None:
                self.joint_limits_lower.append(jnt.limit.lower)
                self.joint_limits_upper.append(jnt.limit.upper)
            else:
                self.joint_limits_lower.append(None)
                self.joint_limits_upper.append(None)
            if jnt.safety_controller is not None:
                self.joint_safety_lower.append(jnt.safety_controller.soft_lower_limit)#.lower)
                self.joint_safety_upper.append(jnt.safety_controller.soft_upper_limit)#.upper)
            elif jnt.limit is not None:
                self.joint_safety_lower.append(jnt.limit.lower)
                self.joint_safety_upper.append(jnt.limit.upper)
            else:
                self.joint_safety_lower.append(None)
                self.joint_safety_upper.append(None)
            self.joint_types.append((jnt.type , jnt.axis))

        def replace_none(x, v):
            if x is None:
                return v
            return x
        self.joint_limits_lower = np.array([replace_none(jl, -np.inf)
                                            for jl in self.joint_limits_lower])
        self.joint_limits_upper = np.array([replace_none(jl, np.inf)
                                            for jl in self.joint_limits_upper])
        self.joint_safety_lower = np.array([replace_none(jl, -np.inf)
                                            for jl in self.joint_safety_lower])
        self.joint_safety_upper = np.array([replace_none(jl, np.inf)
                                            for jl in self.joint_safety_upper])
        self.joint_types = self.joint_types
        self.num_joints = len(self.get_joint_names())
        print '1'
        self._fk_kdl = kdl.ChainFkSolverPos_recursive(self.chain)
        self._ik_v_kdl = kdl.ChainIkSolverVel_pinv(self.chain)
        self._ik_p_kdl = kdl.ChainIkSolverPos_NR(self.chain, self._fk_kdl, self._ik_v_kdl)
        self._jac_kdl = kdl.ChainJntToJacSolver(self.chain)
        self._dyn_kdl = kdl.ChainDynParam(self.chain, kdl.Vector.Zero())

    def extract_joint_state(self, js, joint_names=None):
        if joint_names is None:
            joint_names = self.get_joint_names()
        q   = np.zeros(len(joint_names))
        qd  = np.zeros(len(joint_names))
        eff = np.zeros(len(joint_names))
        for i, joint_name in enumerate(joint_names):
            js_idx = js.name.index(joint_name)
            if js_idx < len(js.position) and q is not None:
                q[i]   = js.position[js_idx]
            else:
                q = None
            if js_idx < len(js.velocity) and qd is not None:
                qd[i]  = js.velocity[js_idx]
            else:
                qd = None
            if js_idx < len(js.effort) and eff is not None:
                eff[i] = js.effort[js_idx]
            else:
                eff = None
        return q, qd, eff

    ##
    # @return List of link names in the kinematic chain.
    def get_link_names(self, joints=False, fixed=True):
        return self.urdf.get_chain(self.base_link, self.end_link, joints, fixed)

    ##
    # @return List of joint names in the kinematic chain.
    def get_joint_names(self, links=False, fixed=False):
        return self.urdf.get_chain(self.base_link, self.end_link,
                                   links=links, fixed=fixed)

    def get_number_joint(self, links=False, fixed=False):
        return self.urdf.get_chain(self.base_link, self.end_link,
                                   links=links, fixed=fixed).size()

    def get_joint_limits(self):
        return self.joint_limits_lower, self.joint_limits_upper

    def FK(self, q, link_number=None):
        if link_number is not None:
            end_link = self.get_link_names(fixed=False)[link_number]
        else:
            end_link = None
        homo_mat = self.forward(q, end_link)
        pos = tf.transformations.translation_from_matrix(q)
        quat = tf.transformations.quaternion_from_matrix(homo_mat)
        euler = tf.transformations.euler_from_matrix(homo_mat)
        return pos, quat, euler


    ##
    # Forward kinematics on the given joint angles, returning the location of the
    # end_link in the base_link's frame.
    # @param q List of joint angles for the full kinematic chain.
    # @param end_link Name of the link the pose should be obtained for.
    # @param base_link Name of the root link frame the end_link should be found in.
    # @return 4x4 numpy.mat homogeneous transformation
    def forward(self, q, end_link=None, base_link=None):
        link_names = self.get_link_names()
        if end_link is None:
            end_link = self.chain.getNrOfSegments()
        else:
            end_link = end_link.split("/")[-1]
            if end_link in link_names:
                end_link = link_names.index(end_link)
            else:
                print "Target segment %s not in KDL chain" % end_link
                return None
        if base_link is None:
            base_link = 0
        else:
            base_link = base_link.split("/")[-1]
            if base_link in link_names:
                base_link = link_names.index(base_link)
            else:
                print "Base segment %s not in KDL chain" % base_link
                return None
        base_trans = self._do_kdl_fk(q, base_link)
        if base_trans is None:
            print "FK KDL failure on base transformation."
        end_trans = self._do_kdl_fk(q, end_link)
        if end_trans is None:
            print "FK KDL failure on end transformation."
        return base_trans * end_trans

    def symbolic_fk(self, q, end_link=None, base_link=None):
        list = self.write_to_list()

        fk = SX.eye(4)
        #print 'symbolic FK'
        for i in range(0, len(list)):
            if list[i][1] == 'revolute':
                rot = self.create_rotation_matrix_sym(q[i],list[i][3])
                print list[i][2]
                fk = mtimes(fk,list[i][2])
                fk = mtimes(fk,rot)
            elif list[i][1] == 'prismatic':
                rospy.loginfo("prismatic")
        print fk
        return fk

    def forward2(self, q, end_link=None, base_link=None):
        list = self.write_to_list()
        print 'forward 2'
        #try:
        #    listner = tf.TransformListener(True, rospy.Duration(40.0))
        #    listner.waitForTransform("base_link", self.base_link, rospy.Time(0), rospy.Duration(0.1))
        #    (trans, quat) = listner.lookupTransform("base_link", self.base_link,rospy.Time(0))
        #    angle = tf.transformations.euler_from_quaternion(quat)
        #except Exception as e:
        #    print('Exception fram does not exist in the kinematic chain: ' + str(e))

        #root_trans_mat = tf.transformations.compose_matrix(angles=angle, translate=trans)

        fk = np.identity(4)
        #print fk
        for i in range(0, len(list)):
            if list[i][1] == 'revolute':
                rot = self.create_rotation_matrix(q[i],list[i][3])
                fk = np.dot(fk, list[i][2])
                fk = np.dot(fk, rot)
                self.tf_list.append((list[i][0],list[i][1],list[i][2],list[i][3],fk))
            elif list[i][1] == 'prismatic':
                rospy.loginfo("prismatic")
        return fk
    def mult_matrix(self, A, B):
        result_mat = np.zeros(shape=(4,4))
        m = 4; n = 4

        for i in range(0, m):
            for j in range(0, n):
                for k in range(0,n):
                    result_mat[i, j] += A[i,k] * B[k,j]

        return result_mat

    # @return rotation matrix of given angle and axis of rotation is z
    def create_rotation_matrix(self, angle,axis):
        rot_mat = np.eye(4, 4)
        #angle = angle * pi / 180  # convert deg to rad

        if axis == [1,0,0]:
            rot_mat = tf.transformations.compose_matrix(angles=[angle,0,0], translate=[0,0,0])
        elif axis == [0,1,0]:
            rot_mat = tf.transformations.compose_matrix(angles=[0,angle, 0], translate=[0, 0, 0])
        elif axis == [0, 0, 1]:
            rot_mat = tf.transformations.compose_matrix(angles=[0, 0,angle], translate=[0, 0, 0])

        return rot_mat

    def create_rotation_matrix_sym(self, angle,axis):
        rot_mat = SX.eye(4)
        #angle = angle * pi / 180  # convert deg to rad
        if axis == [1,0,0]:
            rot_mat[1, 1] = cos(angle)
            rot_mat[1, 2] = -sin(angle)
            rot_mat[2, 1] = sin(angle)
            rot_mat[2, 2] = cos(angle)
        elif axis == [0,1,0]:
            rot_mat[0, 0] = cos(angle)
            rot_mat[0, 2] = -sin(angle)
            rot_mat[2, 0] = sin(angle)
            rot_mat[2, 2] = cos(angle)
        elif axis == [0, 0, 1]:
            rot_mat[0, 0] = cos(angle)
            rot_mat[0, 1] = -sin(angle)
            rot_mat[1, 0] = sin(angle)
            rot_mat[1, 1] = cos(angle)
        return rot_mat

    ##
    # @param Name of joint of which transformation needed
    # @return 4x4 homogeneous transformation
    def create_homo_matrix(self, joint_name):
        joint_names = self.urdf.get_chain(self.base_link, self.end_link, links=False, fixed=False)
        for it in joint_names:
            joint = self.urdf.joint_map[it]
            if joint.name == joint_name:
                angle = joint.origin.rpy
                pose = joint.origin.xyz
                continue
        homo_matrix = tf.transformations.compose_matrix(angles=angle, translate=pose)

        return homo_matrix

    ##
    # Write in the list form
    def write_to_list(self):
        list_fk = []
        trans_wrt_origin = np.identity(4)

        for i, it in enumerate(self.get_joint_names()):
            trans = self.create_homo_matrix(it)
            list_fk.append((it, self.joint_types[i][0], trans,self.joint_types[i][1]))
            #print trans_wrt_origin
        return list_fk

    def _do_kdl_fk(self, q, link_number):
        endeffec_frame = kdl.Frame()
        kinematics_status = self._fk_kdl.JntToCart(self.joint_list_to_kdl(q),
                                                   endeffec_frame,
                                                   link_number)
        print kinematics_status
        if kinematics_status >= 0:
            p = endeffec_frame.p
            M = endeffec_frame.M
            return np.mat([[M[0,0], M[0,1], M[0,2], p.x()],
                           [M[1,0], M[1,1], M[1,2], p.y()],
                           [M[2,0], M[2,1], M[2,2], p.z()],
                           [     0,      0,      0,     1]])
        else:
            return None


    ##
    # Returns the Jacobian matrix at the end_link for the given joint angles.
    # @param q List of joint angles.
    # @return 6xN np.mat Jacobian
    # @param pos Point in base frame where the jacobian is acting on.
    #            If None, we assume the end_link
    def jacobian(self, q, pos=None):
        j_kdl = kdl.Jacobian(self.num_joints)
        q_kdl = self.joint_list_to_kdl(q)
        self._jac_kdl.JntToJac(q_kdl, j_kdl)
        if pos is not None:
            print 'Pos is not NOne'
            ee_pos = self.forward(q)[:3,3]
            pos_kdl = kdl.Vector(pos[0]-ee_pos[0], pos[1]-ee_pos[1],
                                  pos[2]-ee_pos[2])
            j_kdl.changeRefPoint(pos_kdl)
        return self.kdl_to_mat(j_kdl)

    ##
    # Returns the joint space mass matrix at the end_link for the given joint angles.
    # @param q List of joint angles.
    # @return NxN np.mat Inertia matrix
    def inertia(self, q):
        h_kdl = kdl.JntSpaceInertiaMatrix(self.num_joints)
        self._dyn_kdl.JntToMass(self.joint_list_to_kdl(q), h_kdl)
        return self.kdl_to_mat(h_kdl)

    ##
    # Returns the cartesian space mass matrix at the end_link for the given joint angles.
    # @param q List of joint angles.
    # @return 6x6 np.mat Cartesian inertia matrix
    def cart_inertia(self, q):
        H = self.inertia(q)
        J = self.jacobian(q)
        return np.linalg.inv(J * np.linalg.inv(H) * J.T)

    ##
    # Tests to see if the given joint angles are in the joint limits.
    # @param List of joint angles.
    # @return True if joint angles in joint limits.
    def joints_in_limits(self, q):
        lower_lim = self.joint_limits_lower
        upper_lim = self.joint_limits_upper
        return np.all([q >= lower_lim, q <= upper_lim], 0)

    ##
    # Tests to see if the given joint angles are in the joint safety limits.
    # @param List of joint angles.
    # @return True if joint angles in joint safety limits.
    def joints_in_safe_limits(self, q):
        lower_lim = self.joint_safety_lower
        upper_lim = self.joint_safety_upper
        return np.all([q >= lower_lim, q <= upper_lim], 0)

    ##
    # Clips joint angles to the safety limits.
    # @param List of joint angles.
    # @return np.array list of clipped joint angles.
    def clip_joints_safe(self, q):
        lower_lim = self.joint_safety_lower
        upper_lim = self.joint_safety_upper
        return np.clip(q, lower_lim, upper_lim)

    ##
    # @return get joint angles
    def get_joint_angle(self):
        return self.joint_states

    # receives the joint states
    def jointStateCallback(self, msg):
        for i in range(0, len(msg.position)):
            self.joint_states[i] = msg.position[i]

    ##
    # Returns a set of random joint angles distributed uniformly in the safety limits.
    # @return np.array list of random joint angles.
    def random_joint_angles(self):
        lower_lim = self.joint_safety_lower
        upper_lim = self.joint_safety_upper
        lower_lim = np.where(np.isfinite(lower_lim), lower_lim, -np.pi)
        upper_lim = np.where(np.isfinite(upper_lim), upper_lim, np.pi)
        zip_lims = zip(lower_lim, upper_lim)
        return np.array([np.random.uniform(min_lim, max_lim) for min_lim, max_lim in zip_lims])

    ##
    # Returns a difference between the two sets of joint angles while insuring
    # that the shortest angle is returned for the continuous joints.
    # @param q1 List of joint angles.
    # @param q2 List of joint angles.
    # @return np.array of wrapped joint angles for retval = q1 - q2
    def difference_joints(self, q1, q2):
        diff = np.array(q1) - np.array(q2)
        diff_mod = np.mod(diff, 2 * np.pi)
        diff_alt = diff_mod - 2 * np.pi
        for i, continuous in enumerate(self.joint_types[i][0] == 'continuous'):
            if continuous:
                if diff_mod[i] < -diff_alt[i]:
                    diff[i] = diff_mod[i]
                else:
                    diff[i] = diff_alt[i]
        return diff

    def compute_jacobian(self, q):
        #J = np.mat(np.zeros((6, self.num_joints)))
        J = []
        Jvi = np.mat(np.zeros((3, 1)))
        Jwi = np.mat(np.zeros((3, 1)))
        for i in range(0,self.num_joints):
            if i is not 0:

                if self.tf_list[i][1] == 'fixed':
                    rospy.loginfo("Type of joint is fixed")
                elif self.tf_list[i][1] == 'revolute':
                    rospy.loginfo("Type of joint is revolute")
                    axis = self.tf_list[i][3]
                    if axis == [1, 0, 0]:
                        z_i = np.squeeze(np.asarray(self.tf_list[i][4][:3, 0]))
                        o_i = np.squeeze(np.asarray(self.tf_list[i][4][:3, 3]))
                        o_n = np.squeeze(np.asarray(self.tf_list[self.num_joints - 1][4][:3, 3]))
                        # print "child: ", self.tf_list[i][0]
                        # print "joint_end_child: ", self.tf_list[self.num_joints-1][0]
                        # print "joint_start_parent: ", self.tf_list[i-1][0]
                        # print o_i
                        # print "z_i: ", z_i
                        print "Jvi: ", np.cross(z_i, o_n - o_i)
                        Jvi = np.cross(z_i, o_n - o_i)
                        Jwi = z_i
                        J = np.column_stack((J, np.hstack((Jvi, Jwi))))
                    elif axis == [0, 1, 0]:
                        z_i = np.squeeze(np.asarray(self.tf_list[i][4][:3, 1]))
                        o_i = np.squeeze(np.asarray(self.tf_list[i][4][:3, 3]))
                        o_n = np.squeeze(np.asarray(self.tf_list[self.num_joints - 1][4][:3, 3]))
                        print "Jvi: ", np.cross(z_i, o_n - o_i)
                        Jvi = np.cross(z_i, o_n - o_i)
                        Jwi = z_i
                        J = np.column_stack((J, np.hstack((Jvi, Jwi))))
                    elif axis == [0, 0, 1]:
                        z_i = np.squeeze(np.asarray(self.tf_list[i][4][:3, 2]))
                        o_i = np.squeeze(np.asarray(self.tf_list[i][4][:3, 3]))
                        o_n = np.squeeze(np.asarray(self.tf_list[self.num_joints - 1][4][:3, 3]))
                        print "Jvi: ", np.cross(z_i, o_n - o_i)
                        Jvi = np.cross(z_i, o_n - o_i)
                        Jwi = z_i
                        J = np.column_stack((J,np.hstack((Jvi, Jwi))))
                else:
                    rospy.loginfo("Type of joint is other")
            else:
                z0 = np.array([0.0, 0.0, 1.0])
                # z0 = np.squeeze(np.asarray( self.forward_kinematics(q, joint.child, joint.parent)[:3,2]))
                o_i = np.squeeze(np.asarray(self.tf_list[i][4][:3, 3]))
                o_n = np.squeeze(np.asarray(self.tf_list[self.num_joints-1][4][:3, 3]))
                Jvi = np.cross(z0, o_n - o_i)
                J=np.hstack((Jvi,z0))
                print J

        print 'Jacobian'
        print J

    def kdl_to_mat(self,m):
        mat =  np.mat(np.zeros((m.rows(), m.columns())))
        for i in range(m.rows()):
            for j in range(m.columns()):
                mat[i,j] = m[i,j]
        return mat

    def joint_kdl_to_list(self,q):
        if q == None:
            return None
        return [q[i] for i in range(q.rows())]

    def joint_list_to_kdl(self,q):
        if q is None:
            return None
        if type(q) == np.matrix and q.shape[1] == 0:
            q = q.T.tolist()[0]
        q_kdl = kdl.JntArray(len(q))
        for i, q_i in enumerate(q):
            q_kdl[i] = q_i
        return q_kdl

if __name__ == "__main__":

    rospy.init_node("kdl_kinematics")
    if not rospy.is_shutdown():
        #create_kdl_kin("arm_left_base_link", "arm_left_7_link")
        robot = Robot.from_parameter_server()
        kdl_kin = Kinematics(robot, "arm_left_base_link", "arm_left_7_link")
        #q = kdl_kin.random_joint_angles()
        q = [0.0,0.0, 0, 0, 0.00, 0.00,0.0]


        #print kdl_kin.forward(q, "arm_wrist_3_link", "arm_base_link")   # arm_wrist_3_joint
        #print kdl_kin.forward(q, "arm_wrist_2_link", "arm_base_link")  # arm_wrist_2_joint
        #print kdl_kin.forward(q, "arm_wrist_1_link", "arm_base_link")  # arm_wrist_1_link
        #print kdl_kin.forward(q, "arm_forearm_link", "arm_base_link")  # arm_ee_joint
        #print kdl_kin.forward(q, "arm_upper_arm_link", "arm_base_link")    # arm_lift_joint
        #print kdl_kin.forward(q, "arm_shoulder_link", "arm_base_link")  # arm_pan_joint


        pose = kdl_kin.forward2(q)
        print pose
        pose = kdl_kin.forward(q,"arm_left_base_link", "arm_left_7_link")
        print pose
        print kdl_kin.compute_jacobian(q)
        print kdl_kin.jacobian(q)
       # print pose1
        #print kdl_kin.inertia(q)

    else:
        rospy.logerr("Try to again connect ROS")

