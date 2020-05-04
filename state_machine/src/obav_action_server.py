#!/usr/bin/env python

import rospy
from state_machine.msg import ObavAction, ObavFeedback, ObavResult
from utils.sm_utils import AbstractActionServer
from geometry_msgs.msg import Twist, Pose2D
from hw_interface_plugin_roboteq.msg import RoboteqData
import math

class ObavServer(AbstractActionServer):

    count_per_rev_left = 114701
    count_per_rev_right = 120212
    wheel_radius = 0.145
    left_check = 0
    right_check = 0
    ref_count = []
    distance_traveled = [0,0]

    def __init__(self, name):
        AbstractActionServer.__init__(self, name, ObavAction, ObavFeedback, ObavResult)

        #TODO: change subscriber - need to create custom msg class
        self._sub = rospy.Subscriber('avoidance/cmd_vel', Twist, self.obav_cmd_cb, queue_size=1)
        self._pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self._angle = None
        self._distance = None
        self._pose = Pose2D()
        self._nav_vel = Twist()

    def execute_cb(self, goal):
        while self._as.is_active():
            if self._as.is_preempt_requested():
                self._as.set_preempted()

            if self._angle != None and self._distance != None:

                pose_sub = rospy.Subscriber('pose', Pose2D, self.pose_cb, queue_size = 1)

                if self._pose != Pose2D(): #Localization error checking - only run if /pose topic returned data

                    bearing = self._pose.theta + self._angle #goal angular state (current heading + angle passed by OvAv)
                    err = min( (2*math.pi) - abs(bearing - self._pose.theta), 1.0 * abs(bearing - self._pose.theta)) #diff_ang equivalent
                    p_time = rospy.time() #to measure time period
                    p_err = 0 # to measure change in error
                    tot_err = 0 # accumulative error

                    #TODO: change
                    #tweak constants (gain)
                    Kp = 10.0
                    Ki = 2.0
                    Kd = 0.0

                    # Turn rover until it gets within 2 degrees of desired end angle
                    while (err > 2):
                        current_time = rospy.time()
                        dt = current_time - p_time
                        tot_err += err*dt
                        d_err = (err - p_err)/dt
                        PID = Kp*err + Ki*tot_err + Kd*d_err #actuator output

                        self._nav_vel.angular.z = PID
                        self._pub.publish(self._nav_vel)

                        p_err = err
                        p_time = current_time

                        err = min((2*math.pi) - abs(bearing - self._pose.theta), 1.0 * abs(bearing - self._pose.theta))

                    # goal angle reached
                    self._angle = None

                    # Move rover until desired distance reached
                    left_enc_sub = rospy.Subscriber('/roboteq/drivemotorin/left', RoboteqData, self.left_enc_cb, queue_size=1)
                    right_enc_sub = rospy.Subscriber('/roboteq/drivemotorin/right', RoboteqData, self.right_enc_cb, queue_size=1)

                    total_distance = 0

                    while (total_distance < self._distance):
                        self._nav_vel.linear.x = 1.0
                        self._pub.publish(self._nav_vel)
                        total_distance = (0.5) * (self.distance_traveled[0] + self.distance_traveled[1])

                    self._distance = None

                self._as.set_succeeded(self._result())
                self._nav_vel = None

    # ObAv node will publish a custom message with an angle measurement for rover to turn
    # and a distance length to move
    def obav_cmd_cb(self, obav_cmd_msg):
        self._angle = obav_cmd_msg.angle
        self._distance = obav_cmd_msg.distance

    def pose_cb(self, pose):
        self._pose.x = pose.x
        self._pose.y = pose.y

        # correcting angles to values between 0 and 360 (2 pi)
        if pose.theta>2.0*math.pi :
            pose.theta -= 2.0*math.pi
        elif pose.theta < 0:
            pose.theta += 2.0*math.pi

        self._pose.theta = pose.theta

    def left_enc_cb(self, RoboteqData_msg):
        if self.left_check!=0 :
            abs_enc_count = RoboteqData_msg.encoder_counter_absolute[0]
            rel_enc_count = abs_enc_count - self.ref_count[0]
            rel_enc_count = abs_enc_count + self.ref_count[0] if rel_enc_count > 1000000000 else rel_enc_count
            self.ref_count[0] = abs_enc_count

            #revolutions = rel_enc_count//count_per_rev_left
            #theta = (rel_enc_count % count_per_rev_left ) / count_per_rev_left
            #distance_traveled = (revolutions + theta) * 2 * math.pi * wheel_radius

            self.distance_traveled[0] += (1.0*rel_enc_count)/(1.0*ObavServer.count_per_rev_left) * 2 * math.pi * ObavServer.wheel_radius
        else:
            self.ref_count[0] = RoboteqData_msg.encoder_counter_absolute[0]
            self.left_check += 1


    def right_enc_cb(self, RoboteqData_msg):
        if self.right_check!=0 :
            abs_enc_count = RoboteqData_msg.encoder_counter_absolute[0]
            rel_enc_count = abs_enc_count - self.ref_count[1]
            rel_enc_count = abs_enc_count + self.ref_count[1] if rel_enc_count > 1000000000 else rel_enc_count
            self.ref_count[1] = abs_enc_count

            self.distance_traveled[1] += (1.0*rel_enc_count)/(1.0*ObavServer.count_per_rev_right) * 2 * math.pi * ObavServer.wheel_radius
        else:
            self.ref_count[1] = RoboteqData_msg.encoder_counter_absolute[0]
            self.right_check += 1

if __name__ == '__main__':
    rospy.init_node('obav_action_server')
    server = ObavServer('obav_action_server')
    rospy.spin()
