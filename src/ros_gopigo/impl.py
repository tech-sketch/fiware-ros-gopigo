# -*- coding: utf-8 -*-

import rospy

class Gopigo(object):
    def __init__(self, node_name):
        self.node_name = node_name

    def start(self):
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            r.sleep()
