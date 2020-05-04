#!/usr/bin/env python

import rospy
from state_machine.srv import LegSelect, LegSelectResponse
from utils.sm_utils import AbstractService

class LegSelectService(AbstractService):

    def __init__(self, name):
        AbstractService.__init__(self, name, LegSelect)

    def execute_cb(self, goal):
        response = LegSelectResponse()

        leg_number = int(input('Enter the leg number: '))
        num_ids = int(input('Enter the number of aruco markers for this leg: '))
        aruco_ids = []

        for i in range(0, num_ids):
            aruco_ids.append(int(input('Enter id {0}: '.format(i + 1))))

        lat = float(input('Enter the GPS latitutde for this leg: '))
        lon = float(input('Enter the GPS longitude for this leg: '))

        response.leg_number = leg_number
        response.aruco_ids = aruco_ids
        response.gps_coord.latitude = lat
        response.gps_coord.longitude = lon

        return response


if __name__ == '__main__':
    rospy.init_node('leg_select_srv')
    server = LegSelectService('leg_select_srv')
    rospy.spin()
