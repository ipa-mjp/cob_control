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

from CobMPC import CobMPC


joint_pub = JointStatePublisher('/arm/joint_group_velocity_controller/command')
q = Float64MultiArray()

def loop():
    rate = rospy.Rate(100)  # 10hz
    joint_pub.open()
    while not rospy.is_shutdown():
        joint_pub.publish(q)
        rate.sleep()
    thread.exit_thread()

def loop2():
    rate = rospy.Rate(100)  # 10hz
    while not rospy.is_shutdown():
        #print('thread 2')
        rate.sleep()
    thread.exit_thread()

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)

    joint_sub = JointStateSubscriber('/arm/joint_states')
    joint_sub.open()

    #odometry_sub=OdometrySubscriber('/base/odometry_controller/odometry')
    #odometry_sub.open()
    q.data=joint_sub.data.velocity
    # Create two threads as follows
    try:
        #thread.start_new_thread(loop, ())
        #thread.start_new_thread(loop2, ())
        controller = CobMPC(ns="arm")
        controller.spin()
    except:
        print "Error: unable to start thread"

    rospy.spin()
    while not rospy.is_shutdown():
        pass

