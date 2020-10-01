#!/usr/bin/python
# -*- coding: utf-8 -*-
#author;sgl
#date: 20200805

# abstract class

from abc import ABCMeta, abstractmethod
import rospy

class FrameNode(object):
    __metaclass__ = ABCMeta

    def __init__(self):
        self.num = 0

    def init(self):
        self.init_node('base_node')
        self.define_node_publisher()
        self.define_node_subscriber()

    def init_node(self, node_name):
        rospy.init_node(node_name, log_level=rospy.INFO)
        self.define_node_publisher()
        self.define_node_subscriber()
        # img_sub = rospy.Subscriber('/baseCamImage', Image, self.callback_basecam, queue_size=1)

    @abstractmethod
    def define_node_publisher(self):
        pass
    @abstractmethod
    def define_node_subscriber(self):
        pass

    def set_param(self,param, val):
        rospy.set_param(param,val)
    def get_param(self,param):
        return rospy.get_param(param)

    def loginfo(self,info):
        rospy.loginfo(info)
    def logerr(self,info):
        rospy.logerr(info)
    def logwarn(self,info):
        rospy.logwarn(info)


    def Rate(self,rate):
        return rospy.Rate(rate)

    def is_shutdown(self):
        return rospy.is_shutdown()

    @abstractmethod
    def spin(self):
        pass

    def __str__(self):
        return 'basic ros node {}'.format(self.num)

# class Test():

#     @property
#     def _score(self):
#         return self._score
    
#     @property.setter
#     def _score(self,val):
#         self._score = val