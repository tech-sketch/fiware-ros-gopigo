#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import fake_rpi

import unittest

from mock import patch

import rosunit
from geometry_msgs.msg import Twist

sys.modules['RPi'] = fake_rpi.RPi
sys.modules['RPi.GPIO'] = fake_rpi.RPi.GPIO
sys.modules['smbus'] = fake_rpi.smbus


class TestGopigo(unittest.TestCase):
    def setMock(self, mocked_rospy):
        mocked_rospy.get_param.return_value = {
            "mqtt": {
                "host": "testhost",
                "port": 1883,
                "topics": [{
                    "key": "fiware2gopigo",
                    "name": "/mqtt/topics/fiware2gopigo",
                    "re": "^(?P<device_id>.+)@move\\|(?P<cmd>.+)$",
                }, {
                    "key": "fiware2gopigo_exec",
                    "name": "/mqtt/topics/fiware2gopigo_exec",
                    "format": "{device_id}@move|executed {cmd}",
                }],
            },
            "ros": {
                "rate": 20,
                "linear": 0.4,
                "turn": 0.4,
                "circle": {
                    "x": 0.4,
                    "z": 0.3,
                },
                "topics": [{
                    "key": "gopigo",
                    "name": "/ros/topics/gopigo",
                }],
            },
        }

    @patch('fiware_ros_gopigo.gopigo_impl.read_motor_speed')
    @patch('fiware_ros_gopigo.gopigo_impl.motor2')
    @patch('fiware_ros_gopigo.gopigo_impl.motor1')
    @patch('fiware_ros_gopigo.gopigo_impl.rospy')
    def test_init(self, mocked_rospy, mocked_motor1, mocked_motor2, mocked_read_motor_speed):
        from fiware_ros_gopigo.gopigo_impl import Gopigo

        node_name = 'foo'
        self.setMock(mocked_rospy)

        node = Gopigo(node_name)
        self.assertEqual(node.node_name, node_name)

    @patch('fiware_ros_gopigo.gopigo_impl.read_motor_speed')
    @patch('fiware_ros_gopigo.gopigo_impl.motor2')
    @patch('fiware_ros_gopigo.gopigo_impl.motor1')
    @patch('fiware_ros_gopigo.gopigo_impl.rospy')
    def test_start(self, mocked_rospy, mocked_motor1, mocked_motor2, mocked_read_motor_speed):
        from fiware_ros_gopigo.gopigo_impl import Gopigo

        self.setMock(mocked_rospy)
        node = Gopigo('foo')
        node.start()
        mocked_rospy.Subscriber.assert_called_once_with('/ros/topics/gopigo', Twist, node._on_receive)
        mocked_rospy.spin.assert_called_once_with()

        mocked_motor1.assert_called_once_with(False, 0)
        mocked_motor2.assert_called_once_with(False, 0)

    @patch('fiware_ros_gopigo.gopigo_impl.read_motor_speed')
    @patch('fiware_ros_gopigo.gopigo_impl.motor2')
    @patch('fiware_ros_gopigo.gopigo_impl.motor1')
    @patch('fiware_ros_gopigo.gopigo_impl.rospy')
    def test_on_receive_1(self, mocked_rospy, mocked_motor1, mocked_motor2, mocked_read_motor_speed):
        from fiware_ros_gopigo.gopigo_impl import Gopigo

        self.setMock(mocked_rospy)

        data = Twist()
        data.linear.x = 0.01
        data.angular.z = 0.01

        Gopigo('foo')._on_receive(data)

        mocked_motor1.assert_called_once_with(False, 0)
        mocked_motor2.assert_called_once_with(False, 0)
        mocked_read_motor_speed.assert_called_once_with()

    @patch('fiware_ros_gopigo.gopigo_impl.read_motor_speed')
    @patch('fiware_ros_gopigo.gopigo_impl.motor2')
    @patch('fiware_ros_gopigo.gopigo_impl.motor1')
    @patch('fiware_ros_gopigo.gopigo_impl.rospy')
    def test_on_receive_2(self, mocked_rospy, mocked_motor1, mocked_motor2, mocked_read_motor_speed):
        from fiware_ros_gopigo.gopigo_impl import Gopigo

        self.setMock(mocked_rospy)

        data = Twist()
        data.linear.x = 0.8
        data.angular.z = 0.7

        Gopigo('foo')._on_receive(data)

        mocked_motor1.assert_called_once_with(True, 255)
        mocked_motor2.assert_called_once_with(True, 35)
        mocked_read_motor_speed.assert_called_once_with()

    @patch('fiware_ros_gopigo.gopigo_impl.read_motor_speed')
    @patch('fiware_ros_gopigo.gopigo_impl.motor2')
    @patch('fiware_ros_gopigo.gopigo_impl.motor1')
    @patch('fiware_ros_gopigo.gopigo_impl.rospy')
    def test_on_receive_3(self, mocked_rospy, mocked_motor1, mocked_motor2, mocked_read_motor_speed):
        from fiware_ros_gopigo.gopigo_impl import Gopigo

        self.setMock(mocked_rospy)

        data = Twist()
        data.linear.x = -0.4
        data.angular.z = 0.8

        Gopigo('foo')._on_receive(data)

        mocked_motor1.assert_called_once_with(True, 136)
        mocked_motor2.assert_called_once_with(False, 228)
        mocked_read_motor_speed.assert_called_once_with()


if __name__ == '__main__':
    rosunit.unitrun('fiware_ros_gopigo', 'test_gopigo_impl', TestGopigo)
