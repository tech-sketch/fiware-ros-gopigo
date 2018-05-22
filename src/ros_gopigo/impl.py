# -*- coding: utf-8 -*-

import math

from gopigo import motor1, motor2, enc_read, read_motor_speed

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

from ros_gopigo.logging import getLogger
logger = getLogger(__name__)

class Gopigo(object):
    def __init__(self, node_name):
        self.node_name = node_name

    def start(self):
        logger.infof('Gopigo start: {}', self.node_name)

        rospy.Subscriber('cmd_vel', Twist, self._on_receive)
        lwheel = rospy.Publisher("lwheel", Int16, queue_size=1)
        rwheel = rospy.Publisher("rwheel", Int16, queue_size=1)

        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            lwheel.publish(Int16(enc_read(0)))
            rwheel.publish(Int16(enc_read(1)))
            r.sleep()

        motor1(0, 0)
        motor2(0, 0)

        logger.infof('Gopigo stop: {}', self.node_name)

    def _on_receive(self, data):
        logger.infof('received message: {}', data)

        t = data.linear.x
        r = data.angular.z

        logger.debugf('t={}, r={}', t, r)

        mag = math.sqrt(pow(t,2)+pow(r,2))
        if mag < 0.1:
            logger.infof('too small value, linear.x: {}, angular.z: {}, mag: {}', t, r, mag)
            motor1(0, 0)
            motor2(0, 0)

            logger.infof('set moter speed: {}', read_motor_speed())
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

        motor1(m1 >= 0, abs(int(m1 * 255)))
        motor2(m2 >= 0, abs(int(m2 * 255)))

        logger.infof('set moter speed: {}', read_motor_speed())
