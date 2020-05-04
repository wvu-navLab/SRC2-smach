#!/usr/bin/env python

import rospy
from state_machine.srv import Shutdown, ShutdownResponse
from utils.sm_utils import AbstractService

class ShutdownService(AbstractService):

    def __init__(self, name):
        AbstractService.__init__(self, name, Shutdown)

    def execute_cb(self, goal):
        True


if __name__ == '__main__':
    rospy.init_node('shutdown_srv')
    server = ShutdownService('shutdown_srv')
    rospy.spin()
