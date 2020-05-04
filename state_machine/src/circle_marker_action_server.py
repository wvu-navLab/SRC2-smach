#!/usr/bin/env python

import rospy
from state_machine.msg import CircleMarkerAction, CircleMarkerFeedback, CircleMarkerResult
from utils.sm_utils import AbstractActionServer

class CirlceMarkerServer(AbstractActionServer):

    def __init__(self, name):
        AbstractActionServer.__init__(self, name, CircleMarkerAction, CircleMarkerFeedback, CircleMarkerResult)

    def execute_cb(self, goal):
        True


if __name__ == '__main__':
    rospy.init_node('circle_marker_action_server')
    server = CirlceMarkerServer('circle_marker_action_server')
    rospy.spin()
