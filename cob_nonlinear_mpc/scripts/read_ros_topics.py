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

import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import multiprocessing
from subscribers import JointStateSubscriber
from subscribers import OdometrySubscriber
from publishers import JointStatePublisher
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension
import thread
import CobMPC as ctr

from mpc import *


joint_pub = JointStatePublisher('/arm/joint_group_velocity_controller/command')
q = Float64MultiArray()

def loop():

    joint_pub.open()
    while not rospy.is_shutdown():
        #joint_pub.publish(q)
        print("publising")
        rate.sleep()
    thread.exit_thread()

def loop2():
    rate = rospy.Rate(100)  # 10hz
    while not rospy.is_shutdown():
        print('thread 2')
        rate.sleep()
    thread.exit_thread()

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)

    controller = ctr.CobMPC(ns='arm')
    controller.spin()
    rospy.spin()


