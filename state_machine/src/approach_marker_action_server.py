#!/usr/bin/env python

import rospy
from state_machine.msg import ApproachMarkerAction, ApproachMarkerFeedback, ApproachMarkerResult
from utils.sm_utils import AbstractActionServer
from geometry_msgs.msg import Twist

class ApproachMarkerServer(AbstractActionServer):

    def __init__(self, name):
        AbstractActionServer.__init__(self, name, ApproachMarkerAction, ApproachMarkerFeedback, ApproachMarkerResult)
        self._sub = rospy.Subscriber('detection/cmd_vel', Twist, self.cmd_vel_cb, queue_size=1)
        self._pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self._nav_vel = None

    def execute_cb(self, goal):
        while self._as.is_active():
            if self._as.is_preempt_requested():
                self._as.set_preempted()

            if self._nav_vel != None:
                self._pub.publish(self._nav_vel)

                # All cameras have been checked and flag isn't set, so the twist velocity is all
                # zeroes (equivalent to a default Twist object), we're finished
                if self._nav_vel == Twist():
                    self._as.set_succeeded(self._result())

                self._nav_vel = None

    def cmd_vel_cb(self, nav_vel):
        self._nav_vel = nav_vel

if __name__ == '__main__':
    rospy.init_node('approach_marker_action_server')
    server = ApproachMarkerServer('approach_marker_action_server')
    rospy.spin()
