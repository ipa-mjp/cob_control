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
from casadi import *
from setuptools.command.saveopts import saveopts

import rospy
import matplotlib.pyplot as plt
from casadi import *
import thread
import numpy as np
import PyKDL
from urdf_parser_py.urdf import URDF
import PyKDL
from kdl_parser_py.urdf import treeFromUrdfModel
from urdf_parser_py.urdf import Robot
from kdl import Kinematics

class StateConstraints:
    path_constraints_min = 0.0
    path_constraints_max = 0.0
    terminal_constraints_min = 0.0
    terminal_constraints_max = 0.0
    input_constraints_min = 0.0
    input_constraints_max = 0.0

    def __init__(self):
        print("Created state constraints")


class MPC(object):

    def __init__(self, ns):
        self.joint_names = []
        self.shooting_nodes = 0
        self.time_horizon = 0.0
        self.state_dim =0
        self.control_dim = 0
        self.base_active = False
        self.state_constraints_ = StateConstraints()
        self.chain_tip_link = ""
        self.chain_base_link = ""
        self.tracking_frame = ""
        self.join_state_ = np.zeros((self.state_dim,1))
        self.rate = rospy.Rate(100)  # 10hz
        self.thread = None
        #SYBOLIC VARIABLES
        self.x = None

        #SYMBOLIC FUNCTIONS
        self.FK = None

        #KDL
        self.robot = None
        self.kdl_kin = None

        self.init(ns)

    def init(self, ns):

        print 'Initializing MPC...'

        if rospy.has_param(ns + '/joint_names'):
            self.joint_names = rospy.get_param(ns + '/joint_names')
        else:
            rospy.logerr('Parameter joint_names not set');
            # rospy.logwarn('Could not find parameter ~base_dir. Using default base_dir: ' + base_dir)

        if rospy.has_param(ns + '/nmpc/shooting_nodes'):
            self.shooting_nodes = rospy.get_param(ns + '/nmpc/shooting_nodes')
        else:
            rospy.logerr('Parameter shooting_nodes not set')

        if rospy.has_param(ns + '/nmpc/time_horizon'):
            self.time_horizon = rospy.get_param(ns + '/nmpc/time_horizon')
        else:
            rospy.logerr('Parameter time_horizon not set')

        if rospy.has_param(ns + '/nmpc/state_dim'):
            self.state_dim = rospy.get_param(ns + '/nmpc/state_dim')
            self.join_state_ = np.zeros((self.state_dim, 1))
        else:
            rospy.logerr('Parameter state_dim not set')

        if rospy.has_param(ns + '/nmpc/control_dim'):
            self.control_dim = rospy.get_param(ns + '/nmpc/control_dim')
        else:
            rospy.logerr('Parameter control_dim not set')

        if rospy.has_param(ns + '/nmpc/base/base_active'):
            self.base_active = rospy.get_param(ns + '/nmpc/base/base_active')
        else:
            rospy.logerr('Parameter base/base_active not set')

        if rospy.has_param(ns + '/nmpc/constraints/state/path_constraints/min'):
            self.state_constraints_.path_constraints_min = rospy.get_param(ns + '/nmpc/constraints/state/path_constraints/min')
        else:
            rospy.logerr('Parameter state/path_constraints/min not set')

        if rospy.has_param(ns + '/nmpc/constraints/state/path_constraints/max'):
            self.state_constraints_.input_constraints_max = rospy.get_param(ns + '/nmpc/constraints/state/path_constraints/max')
        else:
            rospy.logerr('Parameter state/path_constraints/max not set')

        if rospy.has_param(ns + '/nmpc/constraints/state/terminal_constraints/min'):
            self.state_constraints_.terminal_constraints_min = rospy.get_param(ns + '/nmpc/constraints/state/terminal_constraints/min')
        else:
            rospy.logerr('Parameter state/terminal_constraints/min not set')

        if rospy.has_param(ns + '/nmpc/constraints/state/terminal_constraints/max'):
            self.state_constraints_.terminal_constraints_max = rospy.get_param(ns + '/nmpc/constraints/state/terminal_constraints/max')
        else:
            rospy.logerr('Parameter state/terminal_constraints/max not set')

        if rospy.has_param(ns + '/nmpc/constraints/input/input_constraints/min'):
            self.state_constraints_.input_constraints_min = rospy.get_param(ns + '/nmpc/constraints/input/input_constraints/min')
        else:
            rospy.logerr('Parameter input/input_constraints/min not set')

        if rospy.has_param(ns + '/nmpc/constraints/input/input_constraints/max'):
            self.state_constraints_.input_constraints_max = rospy.get_param(ns + '/nmpc/constraints/input/input_constraints/max')
        else:
            rospy.logerr('Parameter input/input_constraints/max not set')

        if rospy.has_param(ns + '/chain_tip_link'):
            self.chain_tip_link = rospy.get_param(ns +'/chain_tip_link')
        else:
            rospy.logwarn('Could not find parameter chain_tip_link.')
            exit(-1)

        if rospy.has_param(ns + '/chain_base_link'):
            self.chain_base_link = rospy.get_param(ns +'/chain_base_link')
        else:
            rospy.logwarn('Could not find parameter chain_base_link.')
            exit(-1)

        if rospy.has_param(ns + '/frame_tracker/target_frame'):
            self.tracking_frame = rospy.get_param(ns+'/frame_tracker/target_frame')
        else:
            rospy.logwarn('Could not find parameter frame_tracker/target_frame.')
            exit(-2)

        self.robot = Robot.from_parameter_server()
        print 'Initializing Kinematic Chain...'
        self.kdl_kin= Kinematics(self.robot, self.chain_base_link, self.chain_tip_link)
        print 'Done initializing Kinematic Chain...'

        #print 'SYBOLIC VARIABLES'
        self.x = SX.sym('q',self.state_dim,1)
        self.pos = SX.sym('pos',4,4)
        self.fk = self.kdl_kin.symbolic_fk(self.x)
        self.FK = Function('f', [self.x],[self.fk])
        rospy.loginfo("MPC Initialized...")

    def mpcStep(self):
        print("MPC_step")
        print self.FK(self.join_state_)

    def spin(self):
        self.thread=thread.start_new_thread(name="mpc", target=self.mpcStep())
        return

    def symbolic_fk(self):
        T = SX.sym("T", 4, 4)
        self.FK = self.kdl_kin.symbolic_fk(self.x,"arm_base_link", "arm_wrist_3_link")
        if self.base_active:
            T[0, 0] = cos(self.x(2))
            T[0, 1] = -sin(self.x(2))
            T[0, 2] = 0.0
            T[0, 3] = self.x(0)
            T[1, 0] = sin(self.x(2))
            T[1, 1] = cos(self.x(2))
            T[1, 2] = 0.0
            T[1, 3] = self.x(1)
            T[2, 0] = 0.0
            T[2, 1] = 0.0
            T[2, 2] = 1.0
            T[2, 3] = 0
            T[3, 0] = 0.0
            T[3, 1] = 0.0
            T[3, 2] = 0.0
            T[3, 3] = 1.0
            T_base_ = T



