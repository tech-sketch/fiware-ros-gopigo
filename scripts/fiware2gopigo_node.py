#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy

from ros_gopigo.fiware2gopigo_impl import Fiware2Gopigo
from ros_gopigo.logging import get_logger
logger = get_logger(__name__)

NODE_NAME = 'fiware2gopigo'

def main():
    try:
        rospy.init_node(NODE_NAME)
        logger.infof('Start node : {}', NODE_NAME)

        node = Fiware2Gopigo(NODE_NAME)
        node.connect().start()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
