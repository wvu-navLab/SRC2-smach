#include <ros/ros.h>
#include <vector>
#include <algorithm>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int64.h>
#include <geometry_msgs/Pose.h>
#include <pose_update/PoseUpdate.h>
#include <waypoint_gen/GenerateWaypoint.h>
#include <waypoint_nav/SetGoal.h>
#include <waypoint_nav/Interrupt.h>
#include <driving_tools/Stop.h>
#include <driving_tools/MoveForward.h>
#include <driving_tools/CirculateBaseStation.h>
#include <driving_tools/RotateInPlace.h>
#include <volatile_handler/VolatileReport.h>

class SmRd2
{
public:
  // Members -------------------------------------------------------------------------------------------------------------------------
  const unsigned int num_states = 5;
  enum STATE_T {_initialize=0, _planning=1, _traverse=2, _volatile_handler=3, _lost=4};

  // Condition flag declarations
  // bool flag_localized_base = false;
  int flag_localized_base_hauler = 0;
  bool flag_have_true_pose_hauler = false;
  bool flag_waypoint_unreachable_hauler = false;
  bool flag_arrived_at_waypoint_hauler = true;
  double volatile_detected_distance_hauler = -1.0;
  bool flag_localizing_volatile_hauler = false;
  bool flag_volatile_recorded_hauler = false;
  bool flag_volatile_unreachable_hauler = false;
  bool flag_localization_failure_hauler = false;
  bool flag_recovering_localization_hauler = false;
  bool flag_brake_engaged_hauler = false;
  bool flag_fallthrough_condition_hauler = false;

  int flag_localized_base_excavator = 0;
  bool flag_have_true_pose_excavator = false;
  bool flag_waypoint_unreachable_excavator = false;
  bool flag_arrived_at_waypoint_excavator = true;
  double volatile_detected_distance_excavator = -1.0;
  bool flag_localizing_volatile_excavator = false;
  bool flag_volatile_recorded_excavator = false;
  bool flag_volatile_unreachable_excavator = false;
  bool flag_localization_failure_excavator = false;
  bool flag_recovering_localization_excavator = false;
  bool flag_brake_engaged_excavator = false;
  bool flag_fallthrough_condition_excavator = false;

  // State vector
  std::vector<int> state_to_exec; // Only one should be true at a time, if multiple are true then a default state should be executed

  // ROS objects
  ros::NodeHandle nh;
  
  ros::Publisher sm_state_hauler_pub;
  ros::Publisher sm_state_excavator_pub;

  ros::Subscriber localized_base_hauler_sub;
  ros::Subscriber waypoint_unreachable_hauler_sub;
  ros::Subscriber arrived_at_waypoint_hauler_sub;
  ros::Subscriber volatile_detected_hauler_sub;
  ros::Subscriber volatile_recorded_hauler_sub;
  ros::Subscriber localization_failure_hauler_sub;

  ros::Subscriber localized_base_excavator_sub;
  ros::Subscriber waypoint_unreachable_excavator_sub;
  ros::Subscriber arrived_at_waypoint_excavator_sub;
  ros::Subscriber volatile_detected_excavator_sub;
  ros::Subscriber volatile_recorded_excavator_sub;
  ros::Subscriber localization_failure_excavator_sub;

  ros::ServiceClient clt_true_pose_excavator_;
  ros::ServiceClient clt_wp_gen_excavator_;
  ros::ServiceClient clt_wp_nav_set_goal_excavator_;
  ros::ServiceClient clt_wp_nav_interrupt_excavator_;
  ros::ServiceClient clt_vh_report_excavator_;
  ros::ServiceClient clt_stop_excavator_;

  ros::ServiceClient clt_true_pose_hauler_;
  ros::ServiceClient clt_wp_gen_hauler_;
  ros::ServiceClient clt_wp_nav_set_goal_hauler_;
  ros::ServiceClient clt_wp_nav_interrupt_hauler_;
  ros::ServiceClient clt_vh_report_hauler_;
  ros::ServiceClient clt_stop_hauler_;
  // ros::ServiceClient clt_vol_report_;

  // Methods ----------------------------------------------------------------------------------------------------------------------------
  SmRd2(); // Constructor
  void run();
  // State methods
  void stateInitialize();
  void statePlanning();
  void stateTraverse();
  void stateVolatileHandler();
  void stateLost();

  // Subscriber callbacks
  // void localizedBaseCallback(const std_msgs::Bool::ConstPtr& msg);
  void localizedBaseExcavatorCallback(const std_msgs::Int64::ConstPtr& msg);
  void waypointUnreachableExcavatorCallback(const std_msgs::Bool::ConstPtr& msg);
  void arrivedAtWaypointExcavatorCallback(const std_msgs::Bool::ConstPtr& msg);
  void volatileDetectedExcavatorCallback(const std_msgs::Float32::ConstPtr& msg);
  void volatileRecordedExcavatorCallback(const std_msgs::Bool::ConstPtr& msg);
  void localizationFailureExcavatorCallback(const std_msgs::Bool::ConstPtr& msg);

  void localizedBaseHaulerCallback(const std_msgs::Int64::ConstPtr& msg);
  void waypointUnreachableHaulerCallback(const std_msgs::Bool::ConstPtr& msg);
  void arrivedAtWaypointHaulerCallback(const std_msgs::Bool::ConstPtr& msg);
  void volatileDetectedHaulerCallback(const std_msgs::Float32::ConstPtr& msg);
  void volatileRecordedHaulerCallback(const std_msgs::Bool::ConstPtr& msg);
  void localizationFailureHaulerCallback(const std_msgs::Bool::ConstPtr& msg);


};
