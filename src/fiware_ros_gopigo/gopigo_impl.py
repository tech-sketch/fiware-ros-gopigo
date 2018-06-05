# -*- coding: utf-8 -*-

import math

from gopigo import motor1, motor2, read_motor_speed

import rospy
from geometry_msgs.msg import Twist

from fiware_ros_gopigo.params import get_params, find_item
from fiware_ros_gopigo.logging import get_logger
logger = get_logger(__name__)


class Gopigo(object):
    def __init__(self, node_name):
        self.node_name = node_name
        self._params = get_params(rospy.get_param('~'))
        rospy.Subscriber(find_item(self._params.ros.topics, 'key', 'gopigo').name, Twist, self._on_receive)

    def start(self):
        logger.infof('Gopigo start: {}', self.node_name)

        rospy.spin()

        motor1(0, 0)
        motor2(0, 0)

        logger.infof('Gopigo stop: {}', self.node_name)

    def _on_receive(self, data):
        logger.debugf('received message: {}', data)

        t = data.linear.x
        r = data.angular.z

        logger.debugf('t={}, r={}', t, r)

        mag = math.sqrt(pow(t, 2) + pow(r, 2))
        if mag < 0.1:
            logger.debugf('too small value, linear.x: {}, angular.z: {}, mag: {}', t, r, mag)
            motor1(0, 0)
            motor2(0, 0)

            logger.debugf('moter speed: {}', read_motor_speed())
            return

        logger.debugf('mag={}', mag)

        m1 = t/mag * abs(t)
        m2 = t/mag * abs(t)

        logger.debugf('m1={}, m2={}', m1, m2)

        m1 += r/mag * abs(r)
        m2 += -r/mag * abs(r)

        logger.debugf('m1={}, m2={}', m1, m2)

        if m1 != 0:
            m1 = m1/abs(m1) * min(abs(m1), 1.0)
        if m2 != 0:
            m2 = m2/abs(m2) * min(abs(m2), 1.0)

        logger.debugf('m1={}, m2={}', m1, m2)

        motor1(m1 >= 0, abs(int(m1 * 255)))
        motor2(m2 >= 0, abs(int(m2 * 255)))

        logger.debugf('moter speed: {}', read_motor_speed())
