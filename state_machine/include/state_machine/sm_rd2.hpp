#include <ros/ros.h>
#include <vector>
#include <algorithm>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int64.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <waypoint_gen/GenerateWaypoint.h>
#include <waypoint_gen/StartWaypoint.h>
#include <waypoint_nav/SetGoal.h>
#include <waypoint_nav/Interrupt.h>
#include <driving_tools/Stop.h>
#include <driving_tools/MoveForward.h>
#include <driving_tools/CirculateBaseStation.h>
#include <driving_tools/RotateInPlace.h>
#include <volatile_handler/VolatileReport.h>
#include <volatile_handler/ToggleDetector.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <srcp2_msgs/ToggleLightSrv.h>
#include <srcp2_msgs/BrakeRoverSrv.h>
#include <src2_object_detection/ApproachBaseStation.h>
#include <sensor_fusion/RoverStatic.h>
#include <sensor_fusion/GetTruePose.h>
#include <sensor_fusion/HomingUpdate.h>
#include <std_srvs/Empty.h>
#include <waypoint_checker/CheckCollision.h>
#include <boost/bind.hpp>
#include <srcp2_msgs/BrakeRoverSrv.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class SmRd2
{
public:
  // Members -------------------------------------------------------------------------------------------------------------------------
  const unsigned int num_states = 5;
  enum STATE_T {_initialize=0, _planning=1, _traverse=2, _volatile_handler=3, _lost=4};

  // Condition flag declarations
  // bool flag_localized_base = false;
  int flag_localized_base_excavator_ = 0;
  int flag_mobility_excavator_ = 1;
  bool flag_have_true_pose_excavator_ = false;
  bool flag_waypoint_unreachable_excavator_ = false;
  bool flag_arrived_at_waypoint_excavator_ = true;
  bool flag_localizing_volatile_excavator_ = false;
  bool flag_volatile_recorded_excavator_ = false;
  bool flag_volatile_unreachable_excavator_ = false;
  bool flag_localization_failure_excavator_ = false;
  bool flag_recovering_localization_excavator_ = false;
  bool flag_brake_engaged_excavator_ = false;
  bool flag_fallthrough_condition_ = false;
  bool flag_completed_homing_excavator_ = false;
  bool flag_heading_fail_excavator_=false;
  bool need_to_initialize_landmark_excavator_=true;

  int timer_counter = 0;
  double pitch_excavator_ = 0, roll_excavator_ = 0, yaw_excavator_ = 0, yaw_prev_excavator_ = 0;
  double goal_yaw_excavator_;
  bool actionDone_excavator_ = false;
  geometry_msgs::Pose current_pose_excavator_, goal_pose_excavator_;
  geometry_msgs::Point base_location_;
  double waypoint_type_excavator_;
  int driving_mode_excavator_;


  // State vector
  std::vector<int> state_to_exec; // Only one should be true at a time, if multiple are true then a default state should be executed

  // ROS objects
  ros::NodeHandle nh;
  ros::Publisher sm_state_pub_, cmd_vel_pub_excavator_;
  ros::Subscriber localized_base_sub_excavator_;
  ros::Subscriber waypoint_unreachable_sub_excavator_;
  ros::Subscriber arrived_at_waypoint_sub_excavator_;
  ros::Subscriber volatile_detected_sub_excavator_;
  ros::Subscriber volatile_recorded_sub_excavator_;
  ros::Subscriber localization_failure_sub_excavator_;
  ros::Subscriber localization_sub_excavator_;
  ros::Subscriber driving_mode_sub_excavator_;
  ros::Subscriber mobility_sub_excavator_;

  ros::ServiceClient clt_sf_true_pose_excavator_;
  ros::ServiceClient clt_wp_gen_excavator_;
  ros::ServiceClient clt_wp_start_excavator_;
  ros::ServiceClient clt_wp_nav_set_goal_excavator_;
  ros::ServiceClient clt_wp_nav_interrupt_excavator_;
  ros::ServiceClient clt_vh_report_excavator_;
  ros::ServiceClient clt_stop_excavator_;
  ros::ServiceClient clt_rip_excavator_;
  ros::ServiceClient clt_drive_excavator_;
  ros::ServiceClient clt_brake_excavator_;
  ros::ServiceClient clt_lights_excavator_;
  ros::ServiceClient clt_homing_excavator_;
  ros::ServiceClient clt_approach_base_excavator_;
  ros::ServiceClient clt_rover_static_excavator_;
  ros::ServiceClient clt_waypoint_checker_excavator_;
  ros::ServiceClient clt_srcp2_brake_rover_excavator_;

  MoveBaseClient ac_excavator_;
  actionlib::SimpleClientGoalState move_base_state_excavator_;

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
  void localizedBaseCallbackExcavator(const std_msgs::Int64::ConstPtr& msg);
  void mobilityCallbackExcavator(const std_msgs::Int64::ConstPtr& msg);
  void waypointUnreachableCallbackExcavator(const std_msgs::Bool::ConstPtr& msg);
  void arrivedAtWaypointCallbackExcavator(const std_msgs::Bool::ConstPtr& msg);
  void volatileDetectedCallbackExcavator(const std_msgs::Float32::ConstPtr& msg);
  void volatileRecordedCallbackExcavator(const std_msgs::Bool::ConstPtr& msg);
  void localizationFailureCallbackExcavator(const std_msgs::Bool::ConstPtr& msg);
  void localizationCallbackExcavator(const nav_msgs::Odometry::ConstPtr& msg);
  void drivingModeCallbackExcavator(const std_msgs::Int64::ConstPtr& msg);
  void immobilityRecoveryExcavator();
  void homingRecoveryExcavator();

  void setPoseGoalExcavator(move_base_msgs::MoveBaseGoal& poseGoal, double x, double y, double yaw); // m, m, rad
  void doneCallbackExcavator(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result);
  void activeCallbackExcavator();
  void feedbackCallbackExcavator(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);

  void RotateToHeadingExcavator(double desired_yaw);
  void ClearCostmapsExcavator();
  void LightsExcavator(std::string intensity);
  void RotateInPlaceExcavator(double throttle, double time);
  void StopExcavator(double time);
  void DriveExcavator(double throttle, double time);
  void ToggleDetectorExcavator(bool flag);
  void BrakeExcavator(double intensity);
  void RoverStaticExcavator(bool flag);
  void DriveCmdVelExcavator(double vx, double vy, double wz, double time);

};
