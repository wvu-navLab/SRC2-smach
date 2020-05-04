#!/usr/bin/env python

import rospy
from state_machine.msg import GPSNavAction, GPSNavFeedback, GPSNavResult
from utils.sm_utils import AbstractActionServer
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix

class GPSNavServer(AbstractActionServer):

    def __init__(self, name):
        AbstractActionServer.__init__(self, name, GPSNavAction, GPSNavFeedback, GPSNavResult)
        self._sub = rospy.Subscriber('nav/cmd_vel', Twist, self.cmd_vel_cb)
        self._pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self._nav_vel = None

    def execute_cb(self, goal):
        while self._as.is_active():
            if self._as.is_preempt_requested():
                self._as.set_preempted()

            gps_pub = rospy.Publisher('goal_gps_wp', NavSatFix, queue_size=1)
            if goal.use_optimal:
                gps_pub.publish(goal.optimal_gps_coord)
            else:
                gps_pub.publish(goal.gps_coord)

            # Sleep for 3 seconds just to make sure things get going
            rospy.sleep(3.0)

            if self._nav_vel != None:
                self._pub.publish(self._nav_vel)

                # If we've reached the waypoint and the twist velocity is all
                # zeroes (equivalent to a default Twist object), we're finished
                if self._nav_vel == Twist():
                    self._as.set_succeeded(self._result())

                self._nav_vel = None


    def cmd_vel_cb(self, nav_vel):
        self._nav_vel = nav_vel


if __name__ == '__main__':
    rospy.init_node('gps_nav_action_server')
    server = GPSNavServer('gps_nav_action_server')
    rospy.spin()
