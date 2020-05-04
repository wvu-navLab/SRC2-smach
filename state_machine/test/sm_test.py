#!/usr/bin/env python
import rospy, itertools, random
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

def cmd_vel_cb(vel):
    global last_published_vel

    assert vel == last_published_vel

    last_published_vel = None

if __name__ == '__main__':
    rospy.init_node('sm_test_node')

    # Get all the preempt publishers ready
    obav_preempt_pub = rospy.Publisher('obav', Empty, queue_size=1)
    gps_loss_preempt_pub = rospy.Publisher('gps_loss', Empty, queue_size=1)
    comms_loss_preempt_pub = rospy.Publisher('comms_loss', Empty, queue_size=1)
    teleop_preempt_pub = rospy.Publisher('tele_op', Empty, queue_size=1)
    marker_preempt_pub = rospy.Publisher('marker', Empty, queue_size=1)

    # Get all the simulated publishers ready
    gps_nav_pub = rospy.Publisher('nav/cmd_vel', Twist, queue_size=1)
    approach_pub = rospy.Publisher('detection/cmd_vel', Twist, queue_size=1)
    obav_pub = rospy.Publisher('avoidance/cmd_vel', Twist, queue_size=1)
    teleop_pub = rospy.Publisher('teleop/cmd_vel', Twist, queue_size=1)
    comms_loss_pub = rospy.Publisher('comms_loss/cmd_vel', Twist, queue_size=1)
    gps_loss_pub = rospy.Publisher('gps_loss/cmd_vel', Twist, queue_size=1)

    # For testing purposes, we'll have a subscriber whose callback will assert
    # messages are coming in properly.
    rospy.Subscriber('cmd_vel', Twist, cmd_vel_cb)

    # Keep a global variable containing the most recently published velocity.
    # We'll use this to assert the correct velocities are coming through
    last_published_vel = None

    preempt_lists = {
    '/obav': ['/tele_op'],
    '/tele_op': ['/comms_loss'],
    '/gps_loss': ['/tele_op', '/obav'],
    '/comms_loss': ['/obav'],
    '/marker': ['/gps_loss', '/comms_loss', '/tele_op', '/obav']
    }

    # Once we get into GPSNav, we'll begin by testing all the preempt permutations
    preempt_publishers = [
        #obav_preempt_pub,
        (gps_loss_preempt_pub, gps_loss_pub),
        (comms_loss_preempt_pub,  comms_loss_pub),
        (teleop_preempt_pub, teleop_pub),
        (marker_preempt_pub, approach_pub)
    ]

    # This is gonna getty loop-heavy...
    for i in range(1, len(preempt_publishers) + 1):
        current_perms = itertools.permutations(preempt_publishers, i)
        for perm in current_perms:
            rospy.loginfo('Ready to test preempt permutation: {}'.format(tuple([pub[0].name for pub in perm])))
            raw_input('Hit enter to begin test.')
            # This is getting terryfing...
            for p in range(len(perm)):
                pub = perm[p]
                pub[0].publish()
                for j in range(3):
                    vel = Twist()
                    vel.linear.x = random.uniform(1.0, 5.0)

                    last_published_vel = vel
                    pub[1].publish(vel)

                    # Last one?
                    while last_published_vel != None:
                        pass

                    rospy.sleep(0.5)

                if (p != len(perm) - 1) and (perm[p + 1][0].name in preempt_lists[pub[0].name]):
                    raw_input('Hit enter to let {} be preempted by {}.' \
                        .format(pub[0].name, perm[p][0].name))
                else:
                    vel = Twist()

                    last_published_vel = vel
                    pub[1].publish(vel)

                    # Looplooplooplooploop
                    while last_published_vel != None:
                        pass

                    rospy.loginfo('Transitioning back')
