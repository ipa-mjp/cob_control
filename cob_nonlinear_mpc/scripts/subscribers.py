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
import time
import csv
import datetime
import rospy

import abc

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
from geometry_msgs.msg import Pose
import numpy as np

class Subscriber(object):
    """ROS Pzthon subscriber abstract class

        Attributes:
            topic_name: A string representing the topic name to subscribe.
            data_class: The data class of the topic
    """

    __metaclass__ = abc.ABCMeta

    def __init__(self, topic_name, data_class):
        self.topic_name_ = topic_name
        self.data_class_ = data_class
        self.subscriber_ = None
        self.start_ = None
        self.data = data_class()
    def open(self):
        success = True
        self.subscriber_ = rospy.Subscriber(self.topic_name_, self.data_class_, self.callback)
        return success

    @abc.abstractmethod
    def callback(self, data):
        print("HELLO")
        raise NotImplementedError


class JointStateSubscriber(Subscriber):

    def __init__(self, topic_name):
        super(JointStateSubscriber, self).__init__(topic_name, JointState)
        self.init_ = True
        self.joint_positions_ = []
        self.joint_velocities_ = []
        self.data = JointState()

    def callback(self, data):
        self.data=data
        self.joint_positions_ = np.array(data.position)
        self.joint_velocities_ = np.array(data.velocity)
        #rospy.loginfo('joint states')


class OdometrySubscriber(Subscriber):

    def __init__(self, topic_name):
        super(OdometrySubscriber, self).__init__(topic_name, Odometry)
        self.init_ = True
        self.base_position_ = Odometry().pose.pose.position
        self.base_orientation_ = Odometry().pose.pose.orientation
        self.joint_pos_ = np.array([0,0,0])

    def callback(self, data):
        self.base_position_ = data.pose.pose.position
        self.base_orientation_ = data.pose.pose.orientation
        self.convert_orientation_to_joint_pos()
        #rospy.loginfo('odometry')

    def convert_orientation_to_joint_pos(self):
        #Conversion from quaternion to joint angle
        ysqr = self.base_orientation_.y * self.base_orientation_.y
        t3 = 2.0 * (self.base_orientation_.w * self.base_orientation_.z + self.base_orientation_.x * self.base_orientation_.y)
        t4 = 1.0 - 2.0 * (ysqr + self.base_orientation_.z * self.base_orientation_.z)
        self.joint_pos_ = np.array([self.base_position_.x, self.base_position_.y, np.arctan2(t3, t4)])
        return


class FrameTrackerSubscriber(Subscriber):

    def __init__(self, topic_name):
        super(FrameTrackerSubscriber, self).__init__(topic_name, Pose)
        self.init_ = True
        self.pos_ref = Pose.position
        self.q_ref = Pose.orientation

    def callback(self, data):
        self.pos_ref = data.position
        self.q_ref = data.orientation
        #rospy.loginfo('frame_tracker')
