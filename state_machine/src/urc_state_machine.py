#!/usr/bin/env python

import rospy, smach, smach_ros
from sensor_msgs.msg import NavSatFix
from utils.sm_utils import MonitoredMode
from state_machine.msg import ApproachMarkerAction, CircleMarkerAction, CommsLossAction, GPSLossAction, GPSNavAction, \
                                ObavAction, OptimalSearchAction, SignalCompletionAction, TeleOpAction
from state_machine.srv import Init, LegSelect, Shutdown

if __name__ == "__main__":
    rospy.init_node('urc_state_machine')

    sm = smach.StateMachine(
        outcomes=['shutdown', 'preempted', 'aborted']
    )

################################################################################
# Enter the top-level state machine
################################################################################
    with sm:
        smach.StateMachine.add(
            'INIT',
            smach_ros.ServiceState(
                'init_srv',
                Init
            ),
            transitions={
                'succeeded': 'LEG_SELECT'
            }
        )

        smach.StateMachine.add(
            'LEG_SELECT',
            smach_ros.ServiceState(
                'leg_select_srv',
                LegSelect,
                response_slots=['leg_number', 'gps_coord', 'aruco_ids'],
                output_keys=['leg_number', 'gps_coord', 'aruco_ids']
            ),
            transitions={
                'succeeded': 'AUTONOMOUS_NAV'
            },
            remapping={
                'leg_number': 'sm_leg_number',
                'gps_coord': 'sm_gps_coord',
                'aruco_ids': 'sm_aruco_ids'
            }
        )

        smach.StateMachine.add(
            'SHUTDOWN',
            smach_ros.ServiceState(
                'shutdown_srv',
                Shutdown
            ),
            transitions={
                'succeeded': 'shutdown'
            }
        )

        obav_mm = MonitoredMode(
            outcomes=['succeeded'],
            default_outcome='succeeded',
            monitored_topics=['tele_op'],
            outcome_map={
                'succeeded': {
                    'OBAV_HANDLER': 'succeeded'
                }
            }
        )

################################################################################
# Enter the obav monitored mode
################################################################################
        with obav_mm:
            smach.Concurrence.add(
                'OBAV_HANDLER',
                smach_ros.SimpleActionState(
                    'obav_action_server',
                    ObavAction
                )
            )

################################################################################
# Exit the obav monitored mode
################################################################################
        smach.StateMachine.add(
            'OBAV',
            obav_mm,
            transitions={
                'succeeded': 'AUTONOMOUS_NAV',
                'tele_op_preempt': 'TELE_OP'
            }
        )

        gps_loss_mm = MonitoredMode(
            outcomes=['succeeded'],
            default_outcome='succeeded',
            monitored_topics=['tele_op', 'obav'],
            outcome_map={
                'succeeded': {
                    'GPS_LOSS_HANDLER': 'succeeded'
                }
            }
        )

################################################################################
# Enter the GPS loss monitored mode
################################################################################
        with gps_loss_mm:
            smach.Concurrence.add(
                'GPS_LOSS_HANDLER',
                smach_ros.SimpleActionState(
                    'gps_loss_action_server',
                    GPSLossAction
                )
            )

################################################################################
# Exit the GPS loss monitored mode
################################################################################
        smach.StateMachine.add(
            'GPS_LOSS',
            gps_loss_mm,
            transitions={
                'succeeded': 'AUTONOMOUS_NAV',
                'tele_op_preempt': 'TELE_OP',
                'obav_preempt': 'OBAV'
            }
        )

        comms_loss_mm = MonitoredMode(
            outcomes=['succeeded'],
            default_outcome='succeeded',
            monitored_topics=['obav'],
            outcome_map={
                'succeeded': {
                    'COMMS_LOSS_HANDLER': 'succeeded'
                }
            }
        )

################################################################################
# Enter the comms loss monitored mode
################################################################################
        with comms_loss_mm:
            smach.Concurrence.add(
                'COMMS_LOSS_HANDLER',
                smach_ros.SimpleActionState(
                    'comms_loss_action_server',
                    CommsLossAction
                )
            )

################################################################################
# Exit the comms loss monitored mode
################################################################################
        smach.StateMachine.add(
            'COMMS_LOSS',
            comms_loss_mm,
            transitions={
                'succeeded': 'AUTONOMOUS_NAV',
                'obav_preempt': 'OBAV'
            }
        )

        tele_op_mm = MonitoredMode(
            outcomes=['released'],
            default_outcome='released',
            monitored_topics=['comms_loss'],
            outcome_map={
                'released': {
                    'TELE_OP_HANDLER': 'succeeded'
                }
            }
        )

################################################################################
# Enter the teleop monitored mode
################################################################################
        with tele_op_mm:
            smach.Concurrence.add(
                'TELE_OP_HANDLER',
                smach_ros.SimpleActionState(
                    'tele_op_action_server',
                    TeleOpAction
                )
            )

################################################################################
# Exit the teleop monitored mode
################################################################################
        smach.StateMachine.add(
            'TELE_OP',
            tele_op_mm,
            transitions={
                'released': 'AUTONOMOUS_NAV',
                'comms_loss_preempt': 'COMMS_LOSS'
            }
        )

        nav_mm = MonitoredMode(
            outcomes=['marker_found'],
            default_outcome='marker_found',
            outcome_map={
                'marker_found': {
                    'NAV_SM': 'succeeded'
                }
            },
            monitored_topics=['gps_loss', 'comms_loss', 'tele_op', 'obav', 'marker'],
            input_keys=['nmm_gps_coord', 'nmm_aruco_ids']
        )

################################################################################
# Enter the navigation monitored mode
################################################################################
        with nav_mm:
            nav_sm = smach.StateMachine(
                outcomes=['preempted', 'aborted'],
                input_keys=['nsm_gps_coord', 'nsm_aruco_ids']
            )

################################################################################
# Enter the inner navigation state machine
################################################################################
            with nav_sm:
                # Initialize default values in the user data
                nav_sm.userdata.nsm_optimal_gps_coord = NavSatFix()
                nav_sm.userdata.nsm_use_optimal = False

                smach.StateMachine.add(
                    'GPS_NAV',
                    smach_ros.SimpleActionState(
                        'gps_nav_action_server',
                        GPSNavAction,
                        input_keys=['gps_coord', 'optimal_gps_coord', 'use_optimal'],
                        goal_slots=['gps_coord', 'optimal_gps_coord', 'use_optimal']
                    ),
                    transitions={
                        'succeeded': 'OPTIMAL_SEARCH'
                    },
                    remapping={
                        'gps_coord': 'nsm_gps_coord',
                        'optimal_gps_coord': 'nsm_optimal_gps_coord',
                        'use_optimal': 'nsm_use_optimal'
                    }
                )

                # If the GPS nav state manages to reach the provided coordinates
                # without finding any markers, we transition to the optimal
                # search state where some algorithm is used to intelligently
                # generate waypoints that are then passed pack to the GPS nav
                # state.
                smach.StateMachine.add(
                    'OPTIMAL_SEARCH',
                    smach_ros.SimpleActionState(
                        'optimal_search_action_server',
                        OptimalSearchAction,
                        input_keys=['gps_coord', 'aruco_ids'],
                        goal_slots=['gps_coord', 'aruco_ids'],
                        output_keys=['optimal_gps_coord', 'use_optimal'],
                        result_slots=['optimal_gps_coord', 'use_optimal']
                    ),
                    transitions={
                        'succeeded': 'GPS_NAV'
                    },
                    remapping={
                        'gps_coord': 'nsm_gps_coord',
                        'aruco_ids': 'nsm_aruco_ids',
                        'optimal_gps_coord': 'nsm_optimal_gps_coord',
                        'use_optimal': 'nsm_use_optimal'
                    }
                )

################################################################################
# Exit the inner navigation state machine
################################################################################
            smach.Concurrence.add(
                'NAV_SM',
                nav_sm,
                remapping={
                    'nsm_gps_coord': 'nmm_gps_coord',
                    'nsm_aruco_ids': 'nmm_aruco_ids'
                }
            )

################################################################################
# Exit the navigation monitored mode
################################################################################
        smach.StateMachine.add(
            'AUTONOMOUS_NAV',
            nav_mm,
            transitions={
                'marker_found': 'MARKER_HANDLER',
                'gps_loss_preempt': 'GPS_LOSS',
                'comms_loss_preempt': 'COMMS_LOSS',
                'tele_op_preempt': 'TELE_OP',
                'obav_preempt': 'OBAV',
                'marker_preempt': 'MARKER_HANDLER'
            },
            remapping={
                'nmm_gps_coord': 'sm_gps_coord',
                'nmm_aruco_ids': 'sm_aruco_ids'
            }
        )

        marker_mm = MonitoredMode(
            outcomes=['succeeded'],
            default_outcome='succeeded',
            outcome_map={
                'succeeded': {
                    'MARKER_SM': 'succeeded'
                }
            },
            monitored_topics=['gps_loss', 'comms_loss', 'tele_op', 'obav'],
            input_keys=['mmm_aruco_ids']
        )

################################################################################
# Enter the marker detection monitored mode
################################################################################
        with marker_mm:
            marker_sm = smach.StateMachine(
                outcomes=['succeeded', 'preempted', 'aborted'],
                input_keys=['msm_aruco_ids']
            )

################################################################################
# Enter the inner marker approach state machine
################################################################################
            with marker_sm:
                smach.StateMachine.add(
                    'APPROACH_MARKER',
                    smach_ros.SimpleActionState(
                        'approach_marker_action_server',
                        ApproachMarkerAction,
                        input_keys=['aruco_ids']
                    ),
                    transitions={
                        'succeeded': 'CIRCLE_MARKER'
                    },
                    remapping={
                        'aruco_ids': 'msm_aruco_ids'
                    }
                )

                smach.StateMachine.add(
                    'CIRCLE_MARKER',
                    smach_ros.SimpleActionState(
                        'circle_marker_action_server',
                        CircleMarkerAction
                    ),
                    transitions={
                        'succeeded': 'SIGNAL_COMPLETION'
                    }
                )

                smach.StateMachine.add(
                    'SIGNAL_COMPLETION',
                    smach_ros.SimpleActionState(
                        'signal_completion_action_server',
                        SignalCompletionAction
                    ),
                    transitions={
                        'succeeded': 'succeeded'
                    }
                )

################################################################################
# Exit the inner marker approach state machine
################################################################################
            smach.Concurrence.add(
                'MARKER_SM',
                marker_sm,
                remapping={
                    'msm_aruco_ids': 'mmm_aruco_ids'
                }
            )

################################################################################
# Exit the marker detection monitored mode
################################################################################
        smach.StateMachine.add(
            'MARKER_HANDLER',
            marker_mm,
            transitions={
                'succeeded': 'LEG_SELECT',
                'gps_loss_preempt': 'GPS_LOSS',
                'comms_loss_preempt': 'COMMS_LOSS',
                'tele_op_preempt': 'TELE_OP',
                'obav_preempt': 'OBAV'
            },
            remapping={
                'mmm_aruco_ids': 'sm_aruco_ids'
            }
        )

################################################################################
# Exit the top-level state machine
################################################################################

	# Create and start the introspection server
	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
	sis.start()
	rospy.sleep(10)
	# Execute the state machine
	outcome = sm.execute()

	# Wait for ctrl-c to stop the application
	rospy.spin()
	sis.stop()
