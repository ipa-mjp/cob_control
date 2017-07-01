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
        self.jointstate_sub_ = None

    def callback(self, data):
        header = []
        for name in data.name:
            header.append(name + '_position')
        for name in data.name:
            header.append(name + '_velocity')
        print(header)

class OdometrySubscriber(Subscriber):

    def __init__(self, topic_name):
        super(OdometrySubscriber, self).__init__(topic_name, Odometry)
        self.init_ = True
        self.odometry_sub_ = None
    def callback(self, data):
        print("HELLO")