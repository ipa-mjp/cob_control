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
import mpc


class CobMPC(object):

    def __init__(self, ns):
        self.odometry_sub = OdometrySubscriber('/base/odometry_controller/odometry')
        self.joint_sub = JointStateSubscriber('/arm/joint_states')
        self.controller = MPC(ns=ns)

    def spin(self):
        self.joint_sub.open()
        self.odometry_sub.open()