#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy

from ros_gopigo.impl import Gopigo
from ros_gopigo.logging import get_logger
logger = get_logger(__name__)

NODE_NAME = 'ros_gopigo'

def main():
    try:
        rospy.init_node(NODE_NAME)
        logger.infof('Start node : {}', NODE_NAME)

        gopigo = Gopigo(NODE_NAME)
        gopigo.start()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
