#!/usr/bin/python
# -*- coding: utf-8 -*-
from collections import namedtuple

import rospy

from fiware_ros_gopigo.fiware2gopigo_impl import Fiware2Gopigo
from fiware_ros_gopigo.logging import get_logger
logger = get_logger(__name__)

NODE_NAME = 'fiware2gopigo'


def main():
    try:
        rospy.init_node(NODE_NAME)
        logger.infof('Start test node : {}', NODE_NAME)

        msg_type = namedtuple('msg', ('payload',))

        node = Fiware2Gopigo(NODE_NAME)
        r = rospy.Rate(0.5)
        while not rospy.is_shutdown():
            node._on_message(None, None, msg_type(payload='deviceid@move|circle'))
            r.sleep()

    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
