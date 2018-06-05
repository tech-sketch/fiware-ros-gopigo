#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import fake_rpi

import time
import unittest

from mock import patch

import rostest
import rospy

sys.modules['RPi'] = fake_rpi.RPi
sys.modules['RPi.GPIO'] = fake_rpi.RPi.GPIO
sys.modules['smbus'] = fake_rpi.smbus

NODE_NAME = 'ros_gopigo'


class TestGopigo(unittest.TestCase):

    @patch('fiware_ros_gopigo.gopigo_impl.read_motor_speed')
    @patch('fiware_ros_gopigo.gopigo_impl.motor2')
    @patch('fiware_ros_gopigo.gopigo_impl.motor1')
    def test_on_receive(self, mocked_motor1, mocked_motor2, mocked_read_motor_speed):
        from fiware_ros_gopigo.gopigo_impl import Gopigo

        Gopigo(NODE_NAME)

        result = False
        count = 0
        while count < 60:
            if mocked_motor1.called and mocked_motor2.called and mocked_read_motor_speed.called:
                result = True
                break
            count += 1
            time.sleep(1)
        self.assertTrue(result)


if __name__ == '__main__':
    rospy.init_node(NODE_NAME, anonymous=True)
    rostest.rosrun('fiware_ros_gopigo', 'test_gopigo_impl', TestGopigo)
