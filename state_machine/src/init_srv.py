#!/usr/bin/env python

import rospy
from state_machine.srv import Init, InitResponse
from utils.sm_utils import AbstractService

class InitService(AbstractService):

    def __init__(self, name):
        AbstractService.__init__(self, name, Init)

    def execute_cb(self, goal):
        return InitResponse()


if __name__ == '__main__':
    rospy.init_node('init_srv')
    server = InitService('init_srv')
    rospy.spin()
