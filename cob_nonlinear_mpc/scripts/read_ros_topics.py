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
import threading
from subscribers import JointStateSubscriber
from subscribers import OdometrySubscriber
from publishers import JointStatePublisher
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension
import threading

rate = rospy.Rate(10)  # 10hz

def loop(self):
    self.engine.startLoop(False)
    while not rospy.is_shutdown():
        self.engine.iterate()
        rate.sleep()
    self.engine.endLoop()

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    joint_sub = JointStateSubscriber('/arm/joint_states')
    joint_sub.open()
    odometry_sub=OdometrySubscriber('/base/odometry_controller/odometry')

    lock = thread.allocate_lock()

    joint_pub = JointStatePublisher('/arm/joint_group_velocity_controller/command')

    joint_pub.open()
    lock.acquire()  # will block if lock is already held

        lock.release()
        q = Float64MultiArray()
        q.data=joint_sub.data.velocity
        lock.acquire()  # will block if lock is already held
        joint_pub.publish(q)
