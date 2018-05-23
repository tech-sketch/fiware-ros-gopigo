# -*- coding: utf-8 -*-

import os
import ssl
import threading

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

        self.__ros_pub = rospy.Publisher(
            find_item(self._params.ros.topics, 'key', 'gopigo').name,
            Twist,
            queue_size=10)
        
        self.__moving = False
        self.__lock = threading.Lock()

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

    def _do_stop(self):
        with self.__lock:
            self.__moving = False
        logger.debugf('sotp moving')
