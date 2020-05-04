#!/usr/bin/env python

import rospy
from state_machine.msg import SignalCompletionAction, SignalCompletionFeedback, SignalCompletionResult
from utils.sm_utils import AbstractActionServer

class SignalCompletionServer(AbstractActionServer):

    def __init__(self, name):
        AbstractActionServer.__init__(self, name, SignalCompletionAction, SignalCompletionFeedback, SignalCompletionResult)

    def execute_cb(self, goal):
        True


if __name__ == '__main__':
    rospy.init_node('signal_completion_action_server')
    sever = SignalCompletionServer('signal_completion_action_server')
    rospy.spin()
