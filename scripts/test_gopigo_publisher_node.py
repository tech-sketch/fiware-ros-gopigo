#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist

from fiware_ros_gopigo.params import get_params, find_item
from fiware_ros_gopigo.logging import get_logger

logger = get_logger(__name__)

NODE_NAME = 'test_gopigo_publisher_node'


def main():
    try:
        rospy.init_node(NODE_NAME)
        logger.warnf('Start test node : {}', NODE_NAME)
        topic = find_item(get_params(rospy.get_param("~")).ros.topics, 'key', 'gopigo')
        pub = rospy.Publisher(topic.name, Twist, queue_size=1)
        r = rospy.Rate(2)

        twist = Twist()
        twist.linear.x = 0.8
        twist.angular.z = 0.5
        while not rospy.is_shutdown():
            pub.publish(twist)
            r.sleep()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
