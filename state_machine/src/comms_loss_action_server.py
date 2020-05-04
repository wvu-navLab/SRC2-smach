#!/usr/bin/env python

import rospy
from state_machine.msg import CommsLossAction, CommsLossFeedback, CommsLossResult
from utils.sm_utils import AbstractActionServer
from geometry_msgs.msg import Twist

class CommsLossServer(AbstractActionServer):

    def __init__(self, name):
        AbstractActionServer.__init__(self, name, CommsLossAction, CommsLossFeedback, CommsLossResult)
        self._sub = rospy.Subscriber('comms_loss/cmd_vel', Twist, self.cmd_vel_cb, queue_size=1)
        self._pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self._nav_vel = None

    def execute_cb(self, goal):
        while self._as.is_active():
            if self._as.is_preempt_requested():
                self._as.set_preempted()

            if self._nav_vel != None:
                self._pub.publish(self._nav_vel)

                if self._nav_vel == Twist():
                    self._as.set_succeeded(self._result())

                self._nav_vel = None

    def cmd_vel_cb(self, nav_vel):
        self._nav_vel = nav_vel


if __name__ == '__main__':
    rospy.init_node('comms_loss_action_server')
    server = CommsLossServer('comms_loss_action_server')
    rospy.spin()
