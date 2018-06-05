#!/usr/bin/env python
# -*- coding: utf-8 -*-
import unittest
from collections import namedtuple
from math import pi

from mock import MagicMock, patch, call

import rosunit
from geometry_msgs.msg import Twist

from fiware_ros_gopigo.fiware2gopigo_impl import Fiware2Gopigo


class TestFiware2Gopigo(unittest.TestCase):
    def setMock(self, mocked_rospy, mocked_mqtt):
        self.mocked_client = mocked_mqtt.Client.return_value

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

    @patch('fiware_ros_gopigo.fiware2gopigo_impl.mqtt')
    @patch('fiware_ros_gopigo.fiware2gopigo_impl.rospy')
    def test_init(self, mocked_rospy, mocked_mqtt):
        node_name = 'foo'
        self.setMock(mocked_rospy, mocked_mqtt)

        node = Fiware2Gopigo(node_name)
        self.assertEqual(node.node_name, node_name)

        self.assertFalse(mocked_mqtt.called)
        mocked_mqtt.Client.assert_called_once_with(protocol=mocked_mqtt.MQTTv311)
        self.assertEqual(mocked_rospy.on_shutdown.call_count, 3)
        mocked_rospy.Publisher.assert_called_once_with('/ros/topics/gopigo', Twist, queue_size=1)

    @patch('fiware_ros_gopigo.fiware2gopigo_impl.mqtt')
    @patch('fiware_ros_gopigo.fiware2gopigo_impl.rospy')
    def test_connect(self, mocked_rospy, mocked_mqtt):
        self.setMock(mocked_rospy, mocked_mqtt)

        Fiware2Gopigo('foo').connect()
        self.mocked_client.connect.assert_called_once_with('testhost', port=1883, keepalive=60)
        self.mocked_client.loop_start.assert_called_once_with()

    @patch('fiware_ros_gopigo.fiware2gopigo_impl.mqtt')
    @patch('fiware_ros_gopigo.fiware2gopigo_impl.rospy')
    def test_start(self, mocked_rospy, mocked_mqtt):
        self.setMock(mocked_rospy, mocked_mqtt)

        Fiware2Gopigo('foo').start()
        mocked_rospy.spin.assert_called_once_with()

    @patch('fiware_ros_gopigo.fiware2gopigo_impl.mqtt')
    @patch('fiware_ros_gopigo.fiware2gopigo_impl.rospy')
    def test_on_connect(self, mocked_rospy, mocked_mqtt):
        self.setMock(mocked_rospy, mocked_mqtt)

        Fiware2Gopigo('foo')._on_connect(self.mocked_client, None, None, 0)
        self.mocked_client.subscribe.assert_called_once_with('/mqtt/topics/fiware2gopigo')

    @patch('fiware_ros_gopigo.fiware2gopigo_impl.mqtt')
    @patch('fiware_ros_gopigo.fiware2gopigo_impl.rospy')
    def test_on_message(self, mocked_rospy, mocked_mqtt):
        self.setMock(mocked_rospy, mocked_mqtt)

        msg_type = namedtuple('msg', ('payload',))

        sender = Fiware2Gopigo('foo')

        sender._do_circle = MagicMock()
        sender._on_message(self.mocked_client, None, msg_type(payload='deviceid@move|circle'))
        sender._do_circle.assert_called_once_with()

        cmdexec = 'deviceid@move|executed circle'
        self.mocked_client.publish.assert_called_once_with('/mqtt/topics/fiware2gopigo_exec', cmdexec)

        sender._do_circle = MagicMock()
        sender._on_message(self.mocked_client, None, msg_type(payload='invalid'))
        sender._do_circle.assert_not_called()
        self.assertEqual(self.mocked_client.publish.call_count, 1)

    @patch('fiware_ros_gopigo.fiware2gopigo_impl.mqtt')
    @patch('fiware_ros_gopigo.fiware2gopigo_impl.rospy')
    def test_do_circle(self, mocked_rospy, mocked_mqtt):
        self.setMock(mocked_rospy, mocked_mqtt)

        mocked_pub = mocked_rospy.Publisher.return_value

        t = Fiware2Gopigo('foo')._do_circle()
        t.join()
        mocked_rospy.Rate.assert_called_once_with(20)
        self.assertEqual(mocked_pub.publish.call_count, 2 * int(2 * pi * 20) + 1)
        args_list = mocked_pub.publish.call_args_list
        twist = Twist()
        twist.linear.x = 0.4
        twist.angular.z = 0.3
        for i in range(2 * int(2 * pi * 20)):
            self.assertEqual(args_list[i], call(twist))
        self.assertEqual(args_list[-1], call(Twist()))

    @patch('fiware_ros_gopigo.fiware2gopigo_impl.mqtt')
    @patch('fiware_ros_gopigo.fiware2gopigo_impl.rospy')
    def test_do_square(self, mocked_rospy, mocked_mqtt):
        self.setMock(mocked_rospy, mocked_mqtt)

        mocked_pub = mocked_rospy.Publisher.return_value

        t = Fiware2Gopigo('foo')._do_square()
        t.join()
        mocked_rospy.Rate.assert_called_with(20)
        self.assertEqual(mocked_pub.publish.call_count, 4 * (2 * 20 + 1) + 4 * (int(pi / 2 * 20) + 1))
        args_list = mocked_pub.publish.call_args_list
        linear = Twist()
        linear.linear.x = 0.4
        linear.angular.z = 0.0
        rotate = Twist()
        rotate.linear.x = 0.4
        rotate.angular.z = 0.4
        for i in range(2 * 20):
            self.assertEqual(args_list[i], call(linear))
        self.assertEqual(args_list[i + 1], call(Twist()))
        for j in range(int(pi / 2 * 20)):
            self.assertEqual(args_list[i + 2 + j], call(rotate))
        self.assertEqual(args_list[i + 2 + j + 1], call(Twist()))
        self.assertEqual(args_list[-1], call(Twist()))

    @patch('fiware_ros_gopigo.fiware2gopigo_impl.mqtt')
    @patch('fiware_ros_gopigo.fiware2gopigo_impl.rospy')
    def test_do_triangle(self, mocked_rospy, mocked_mqtt):
        self.setMock(mocked_rospy, mocked_mqtt)

        mocked_pub = mocked_rospy.Publisher.return_value

        t = Fiware2Gopigo('foo')._do_triangle()
        t.join()
        mocked_rospy.Rate.assert_called_with(20)
        self.assertEqual(mocked_pub.publish.call_count, 3 * (2 * 20 + 1) + 3 * (int(pi * 2 / 3 * 20) + 1))
        args_list = mocked_pub.publish.call_args_list
        linear = Twist()
        linear.linear.x = 0.4
        linear.angular.z = 0.0
        rotate = Twist()
        rotate.linear.x = 0.4
        rotate.angular.z = 0.4
        for i in range(2 * 20):
            self.assertEqual(args_list[i], call(linear))
        self.assertEqual(args_list[i + 1], call(Twist()))
        for j in range(int(pi * 2 / 3 * 20)):
            self.assertEqual(args_list[i + 2 + j], call(rotate))
        self.assertEqual(args_list[i + 2 + j + 1], call(Twist()))
        self.assertEqual(args_list[-1], call(Twist()))

    @patch('fiware_ros_gopigo.fiware2gopigo_impl.mqtt')
    @patch('fiware_ros_gopigo.fiware2gopigo_impl.rospy')
    def test_do_forward(self, mocked_rospy, mocked_mqtt):
        self.setMock(mocked_rospy, mocked_mqtt)

        mocked_pub = mocked_rospy.Publisher.return_value

        t = Fiware2Gopigo('foo')._do_forward()
        t.join()
        mocked_rospy.Rate.assert_called_with(20)
        self.assertEqual(mocked_pub.publish.call_count, int(0.4 * 20) + 1)
        args_list = mocked_pub.publish.call_args_list
        linear = Twist()
        linear.linear.x = 0.4
        linear.angular.z = 0.0
        for i in range(int(0.4 * 20)):
            self.assertEqual(args_list[i], call(linear))
        self.assertEqual(args_list[-1], call(Twist()))

    @patch('fiware_ros_gopigo.fiware2gopigo_impl.mqtt')
    @patch('fiware_ros_gopigo.fiware2gopigo_impl.rospy')
    def test_do_backward(self, mocked_rospy, mocked_mqtt):
        self.setMock(mocked_rospy, mocked_mqtt)

        mocked_pub = mocked_rospy.Publisher.return_value

        t = Fiware2Gopigo('foo')._do_backward()
        t.join()
        mocked_rospy.Rate.assert_called_with(20)
        self.assertEqual(mocked_pub.publish.call_count, int(0.4 * 20) + 1)
        args_list = mocked_pub.publish.call_args_list
        linear = Twist()
        linear.linear.x = -0.4
        linear.angular.z = 0.0
        for i in range(int(0.4 * 20)):
            self.assertEqual(args_list[i], call(linear))
        self.assertEqual(args_list[-1], call(Twist()))

    @patch('fiware_ros_gopigo.fiware2gopigo_impl.mqtt')
    @patch('fiware_ros_gopigo.fiware2gopigo_impl.rospy')
    def test_do_turnleft(self, mocked_rospy, mocked_mqtt):
        self.setMock(mocked_rospy, mocked_mqtt)

        mocked_pub = mocked_rospy.Publisher.return_value

        t = Fiware2Gopigo('foo')._do_turnleft()
        t.join()
        mocked_rospy.Rate.assert_called_with(20)
        self.assertEqual(mocked_pub.publish.call_count, (int(pi / 18 * 20) + 1))
        args_list = mocked_pub.publish.call_args_list
        rotate = Twist()
        rotate.linear.x = 0.4
        rotate.angular.z = 0.4
        for i in range(int(pi / 18 * 20)):
            self.assertEqual(args_list[i], call(rotate))
        self.assertEqual(args_list[-1], call(Twist()))

    @patch('fiware_ros_gopigo.fiware2gopigo_impl.mqtt')
    @patch('fiware_ros_gopigo.fiware2gopigo_impl.rospy')
    def test_do_turnright(self, mocked_rospy, mocked_mqtt):
        self.setMock(mocked_rospy, mocked_mqtt)

        mocked_pub = mocked_rospy.Publisher.return_value

        t = Fiware2Gopigo('foo')._do_turnright()
        t.join()
        mocked_rospy.Rate.assert_called_with(20)
        self.assertEqual(mocked_pub.publish.call_count, (int(pi / 18 * 20) + 1))
        args_list = mocked_pub.publish.call_args_list
        rotate = Twist()
        rotate.linear.x = 0.4
        rotate.angular.z = -0.4
        for i in range(int(pi / 18 * 20)):
            self.assertEqual(args_list[i], call(rotate))
        self.assertEqual(args_list[-1], call(Twist()))


if __name__ == '__main__':
    rosunit.unitrun('fiware_ros_gopigo', 'test_fiware2gopigo_impl', TestFiware2Gopigo)
