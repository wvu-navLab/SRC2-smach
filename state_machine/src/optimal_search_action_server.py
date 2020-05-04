#!/usr/bin/env python

import rospy, random
from sensor_msgs.msg import NavSatFix
from state_machine.msg import OptimalSearchAction, OptimalSearchFeedback, OptimalSearchResult
from utils.sm_utils import AbstractActionServer

class OptimalSearchServer(AbstractActionServer):

    def __init__(self, name):
        AbstractActionServer.__init__(self, name, OptimalSearchAction, OptimalSearchFeedback, OptimalSearchResult)

    def execute_cb(self, goal):
        while self._as.is_active():
            if self._as.is_preempt_requested():
                self._as.set_preempted()

            generated_lat = random.gauss(goal.gps_coord.latitude, 0.0001)
            generated_lon = random.gauss(goal.gps_coord.longitude, 0.0001)

            optimal_coord = NavSatFix()
            optimal_coord.latitude = generated_lat
            optimal_coord.longitude = generated_lon

            result_msg = self._result()
            result_msg.optimal_gps_coord = optimal_coord
            result_msg.use_optimal = True

            self._as.set_succeeded(result_msg)


if __name__ == '__main__':
    rospy.init_node('optimal_search_action_server')
    sever = OptimalSearchServer('optimal_search_action_server')
    rospy.spin()
