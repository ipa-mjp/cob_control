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

from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
import tf

class Publisher(object):
    """ROS Pzthon subscriber abstract class

        Attributes:
            topic_name: A string representing the topic name to subscribe.
            data_class: The data class of the topic
    """

    __metaclass__ = abc.ABCMeta

    def __init__(self, topic_name, data_class):
        self.topic_name_ = topic_name
        self.data_class_ = data_class
        self.publisher_ = None
        self.start_ = None

    def open(self):
        success = True
        self.publisher_ = rospy.Publisher(self.topic_name_, self.data_class_)
        return success

    @abc.abstractmethod
    def publish(self, data):
        print("HELLO")
        raise NotImplementedError

class JointStatePublisher(Publisher):

    def __init__(self, topic_name):
        super(JointStatePublisher, self).__init__(topic_name, Float64MultiArray)
        self.init_ = True

    def publish(self, data):
        self.publisher_.publish(data)
        print("Hello")

class OdometryPublisher(Publisher):

    def __init__(self, topic_name):
        super(OdometryPublisher, self).__init__(topic_name, Odometry)
        self.init_ = True

    def publish(self, data):
        self.publisher_.publish(data)