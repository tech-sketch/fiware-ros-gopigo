# -*- coding: utf-8 -*-

import os
import re
import ssl
import threading
from math import pi

import paho.mqtt.client as mqtt

import rospy
from geometry_msgs.msg import Twist

from ros_gopigo.params import get_params, find_item
from ros_gopigo.logging import get_logger
logger = get_logger(__name__)

class Fiware2Gopigo(object):
    def __init__(self, node_name):
        self.node_name = node_name
        self._params = get_params(rospy.get_param('~'))

        self.__client = mqtt.Client(protocol=mqtt.MQTTv311)
        self.__client.on_connect = self._on_connect
        self.__client.on_message = self._on_message

        rospy.on_shutdown(self._do_stop)
        rospy.on_shutdown(self.__client.disconnect)
        rospy.on_shutdown(self.__client.loop_stop)

        self.__ros_pub = rospy.Publisher(find_item(self._params.ros.topics, 'key', 'gopigo').name,
                                         Twist,
                                         queue_size=1)
        
        self.__moving = False
        self.__lock = threading.Lock()

        self._cmd_payload_re = re.compile(find_item(self._params.mqtt.topics, 'key', 'fiware2gopigo').re)

    def connect(self):
        logger.infof('Connect to MQTT broker')

        if hasattr(self._params.mqtt, 'cafile'):
            cafile_path = self._params.mqtt.cafile.strip()
            if len(cafile_path) > 0 and os.path.isfile(cafile_path):
                self.__client.tls_set(cafile_path, tls_version=ssl.PROTOCOL_TLSv1_2)

        if hasattr(self._params.mqtt, 'username') and hasattr(self._params.mqtt, 'password'):
            username = self._params.mqtt.username.strip()
            password = self._params.mqtt.password.strip()
            if len(username) > 0 and len(password) > 0:
                self.__client.username_pw_set(username, password)

        self.__client.connect(self._params.mqtt.host, port=self._params.mqtt.port, keepalive=60)
        self.__client.loop_start()
        return self

    def start(self):
        logger.infof('Fiware2Gopigo start: {}', self.node_name)
        rospy.spin()
        logger.infof('Fiware2Gopigo stop: {}', self.node_name)

    def _on_connect(self, client, userdata, flags, response_code):
        logger.infof('connected to MQTT Broker, status: {}', response_code)
        client.subscribe(find_item(self._params.mqtt.topics, 'key', 'fiware2gopigo').name)

    def _on_message(self, client, userdata, msg):
        payload = str(msg.payload)
        logger.infof('received message from mqtt: {}', payload)
        matcher = self._cmd_payload_re.match(payload)
        if matcher:
            cmd = matcher.group('cmd')
            device_id = matcher.group('device_id')
            if cmd == 'circle':
                self._do_circle()
            elif cmd == 'square':
                self._do_square()
            elif cmd == 'triangle':
                self._do_triangle()
            elif cmd == 'cross':
                self._do_stop()
            elif cmd == 'up':
                self._do_forward()
            elif cmd == 'down':
                self._do_backward()
            elif cmd == 'left':
                self._do_turnleft()
            elif cmd == 'right':
                self._do_turnright()
            else:
                logger.warnf('unknown cmd: {}', payload)
                cmd = 'UNKNOWN CMD: {}'.format(cmd)
            topic = find_item(self._params.mqtt.topics, 'key', 'fiware2gopigo_exec').name
            fmt = find_item(self._params.mqtt.topics, 'key', 'fiware2gopigo_exec').format
            self.__client.publish(topic, fmt.format(device_id=device_id, cmd=cmd))
        else:
            logger.warnf('unkown payload: {}', payload)
        self.__ros_pub.publish(Twist())
        logger.debugf('active threds = {}', threading.active_count())

    def _do_stop(self):
        logger.infof('stop moving')
        with self.__lock:
            self.__moving = False
        self.__ros_pub.publish(Twist())

    def _do_circle(self):
        logger.infof('do circle')
        def move(self):
            self.__circle(int(2 * pi * self._params.ros.rate))
        return self._do_move(move)

    def _do_square(self):
        logger.infof('do square')
        def move(self):
            self.__linear(2 * self._params.ros.rate)
            self.__rotate(pi / 2)
            self.__linear(2 * self._params.ros.rate)
            self.__rotate(pi / 2)
            self.__linear(2 * self._params.ros.rate)
            self.__rotate(pi / 2)
            self.__linear(2 * self._params.ros.rate)
            self.__rotate(pi / 2)
        return self._do_move(move)

    def _do_triangle(self):
        logger.infof('do triangle')
        def move(self):
            self.__linear(2 * self._params.ros.rate)
            self.__rotate(pi * 2 / 3)
            self.__linear(2 * self._params.ros.rate)
            self.__rotate(pi * 2 / 3)
            self.__linear(2 * self._params.ros.rate)
            self.__rotate(pi * 2 / 3)
        return self._do_move(move)

    def _do_forward(self):
        logger.infof('do forward')
        def move(self):
            self.__linear(int(self._params.ros.rate * 0.4))
        return self._do_move(move)

    def _do_backward(self):
        logger.infof('do backward')
        def move(self):
            self.__linear(int(self._params.ros.rate * 0.4), reverse=True)
        return self._do_move(move)

    def _do_turnleft(self):
        logger.infof('do turn left')
        def move(self):
            self.__rotate(pi / 18)
        return self._do_move(move)

    def _do_turnright(self):
        logger.infof('do turn right')
        def move(self):
            self.__rotate(pi / 18, reverse=True)
        return self._do_move(move)

    def _do_move(self, callback):
        def func():
            if not callable(callback):
                return

            if self.__moving:
                logger.infof('now moving')
                return

            with self.__lock:
                self.__moving = True

            callback(self)

            with self.__lock:
                self.__moving = False
        thread = threading.Thread(target=func)
        thread.start()
        return thread

    def __circle(self, ticks):
        move_cmd = Twist()
        move_cmd.linear.x = 1.0
        move_cmd.angular.z = 0.8
        self.__move(ticks * 2.0, move_cmd)

    def __linear(self, ticks, reverse=False):
        move_cmd = Twist()
        linear_x = self._params.ros.linear.x
        move_cmd.linear.x = 1.0 if not reverse else -1.0
        self.__move(ticks, move_cmd)

    def __rotate(self, angle, reverse=False):
        move_cmd = Twist()
        angular_z = self._params.ros.angular.z
        move_cmd.angular.z = 0.8 if not reverse else 0.8
        ticks = angle * self._params.ros.rate
        self.__move(ticks, move_cmd)

    def __move(self, ticks, move_cmd):
        r = rospy.Rate(self._params.ros.rate)
        for t in range(int(ticks/ 8.0)):
            if not self.__moving:
                self.__ros_pub.publish(Twist())
                break
            self.__ros_pub.publish(move_cmd)
            r.sleep()
        self.__ros_pub.publish(Twist())
