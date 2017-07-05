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
from subscribers import JointStateSubscriber
from subscribers import OdometrySubscriber
import thread
from mpc import MPC


class CobMPC(object):

    def __init__(self, ns):
        self.odometry_sub = OdometrySubscriber('/base/odometry_controller/odometry')
        self.joint_sub = JointStateSubscriber('/arm/joint_states')
        self.controller = MPC(ns=ns)
        self.thread = None
        self.rate= rospy.Rate(100)  # 10hz

    def spin(self):
        self.thread = thread.start_new_thread(name="cob_mpc", target=self.loop())

    def loop(self):
        self.joint_sub.open()
        self.odometry_sub.open()
        #self.controller.spin()
        while not rospy.is_shutdown():
            self.getJointState()
            self.rate.sleep()
            pass
        self.controller.thread.exit_thread()
        self.thread.thread.exit_thread()
        rospy.loginfo('cob_mpc loop')
        return

    def getJointState(self):
        if self.controller.base_active:
            self.controller.join_state_=np.array([self.odometry_sub.joint_pos_,self.joint_sub.joint_positions_])
            rospy.loginfo(self.joint_sub.joint_positions_)
        else:
            self.controller.join_state_ = self.joint_sub.joint_positions_
        return