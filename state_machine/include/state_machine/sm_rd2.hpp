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
#include <move_excavator/ExcavationStatus.h>
#include <round2_volatile_handler/NextVolatileLocation.h>
#include <sensor_fusion/SetBaseLocation.h>
#include <range_to_base/LocationOfBase.h>


#include <move_excavator/HomeArm.h>
#include <move_excavator/DigVolatile.h>
#include <move_excavator/Scoop.h>
#include <move_excavator/ExtendArm.h>
#include <move_excavator/DropVolatile.h>
#include <move_excavator/ExcavatorFK.h>


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
  bool flag_volatile_dug_excavator_ = true;

  int flag_localized_base_hauler_ = 0;
  int flag_mobility_hauler_ = 1;
  bool flag_have_true_pose_hauler_ = false;
  bool flag_waypoint_unreachable_hauler_ = false;
  bool flag_arrived_at_waypoint_hauler_ = true;
  bool flag_localizing_volatile = false;
  bool flag_volatile_unreachable = false;
  bool flag_localization_failure_hauler_ = false;
  bool flag_recovering_localization_hauler_ = false;
  bool flag_brake_engaged_hauler_ = false;
  bool flag_fallthrough_condition_hauler_ = false;
  bool flag_completed_homing_hauler_ = false;
  bool flag_heading_fail_hauler_=false;
  bool need_to_initialize_landmark_hauler_ = true;

  std::string goal_vol_type_;
  geometry_msgs::Pose goal_vol_pose_;

  int timer_counter = 0;
  double pitch_excavator_ = 0, roll_excavator_ = 0, yaw_excavator_ = 0, yaw_prev_excavator_ = 0;
  double goal_yaw_excavator_;
  bool actionDone_excavator_ = false;
  geometry_msgs::Pose current_pose_excavator_, goal_pose_excavator_;
  geometry_msgs::Point base_location_;
  double waypoint_type_excavator_;
  int driving_mode_excavator_;
  bool excavation_finished_excavator_;
  double collected_mass_excavator_;



  int timer_counter_hauler_ = 0;
  double pitch_hauler_ = 0, roll_hauler_ = 0, yaw_hauler_ = 0, yaw_prev_hauler_ = 0;
  double goal_yaw_hauler_;
  bool actionDone_hauler_ = false;
  geometry_msgs::Pose current_pose_hauler_, goal_pose_hauler_;
  double waypoint_type_hauler_;
  int driving_mode_hauler_;


  // State vector
  std::vector<int> state_to_exec; // Only one should be true at a time, if multiple are true then a default state should be executed

  // ROS objects
  ros::NodeHandle nh;
  ros::Publisher sm_state_pub_, cmd_vel_pub_excavator_;
  ros::Publisher manip_state_pub_excavator_;
  ros::Publisher manip_volatile_pose_pub_excavator_;

  ros::Subscriber localized_base_sub_excavator_;
  ros::Subscriber waypoint_unreachable_sub_excavator_;
  ros::Subscriber arrived_at_waypoint_sub_excavator_;
  ros::Subscriber volatile_detected_sub_excavator_;
  ros::Subscriber volatile_recorded_sub_excavator_;
  ros::Subscriber localization_failure_sub_excavator_;
  ros::Subscriber localization_sub_excavator_;
  ros::Subscriber driving_mode_sub_excavator_;
  ros::Subscriber mobility_sub_excavator_;
  ros::Subscriber manip_feedback_sub_excavator_;

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
  ros::ServiceClient clt_next_vol_excavator_;
  ros::ServiceClient clt_srcp2_brake_rover_excavator_;
  ros::ServiceClient clt_home_arm_excavator_;
  ros::ServiceClient clt_dig_volatile_excavator_;
  ros::ServiceClient clt_scoop_excavator_;
  ros::ServiceClient clt_extend_arm_excavator_ ;
  ros::ServiceClient clt_drop_volatile_excavator_;
  ros::ServiceClient clt_set_base_excavator_ ;
  ros::ServiceClient clt_location_of_base_excavator_;

  // HAULER
  ros::Publisher cmd_vel_pub_hauler_;
  ros::Subscriber localized_base_sub_hauler_;
  ros::Subscriber waypoint_unreachable_sub_hauler_;
  ros::Subscriber arrived_at_waypoint_sub_hauler_;
  ros::Subscriber localization_failure_sub_hauler_;
  ros::Subscriber localization_sub_hauler_;
  ros::Subscriber driving_mode_sub_hauler_;
  ros::Subscriber mobility_sub_hauler_;
  ros::ServiceClient clt_sf_true_pose_hauler_;
  ros::ServiceClient clt_wp_gen_hauler_;
  ros::ServiceClient clt_wp_start_hauler_;
  ros::ServiceClient clt_wp_nav_set_goal_hauler_;
  ros::ServiceClient clt_wp_nav_interrupt_hauler_;
  ros::ServiceClient clt_vh_report_hauler_;
  ros::ServiceClient clt_stop_hauler_;
  ros::ServiceClient clt_rip_hauler_;
  ros::ServiceClient clt_drive_hauler_;
  ros::ServiceClient clt_vol_report_hauler_;
  ros::ServiceClient clt_vol_detect_hauler_;
  ros::ServiceClient clt_brake_hauler_;
  ros::ServiceClient clt_lights_hauler_;
  ros::ServiceClient clt_homing_hauler_;
  ros::ServiceClient clt_approach_base_hauler_;
  ros::ServiceClient clt_approach_excavator_hauler_;
  ros::ServiceClient clt_rover_static_hauler_;
  ros::ServiceClient clt_waypoint_checker_hauler_;
  ros::ServiceClient clt_srcp2_brake_rover_hauler_;
  ros::ServiceClient clt_set_base_hauler_ ;
  ros::ServiceClient clt_location_of_base_hauler_;


  MoveBaseClient ac_excavator_;
  actionlib::SimpleClientGoalState move_base_state_excavator_;
  MoveBaseClient ac_hauler_;
  actionlib::SimpleClientGoalState move_base_state_hauler_;


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
  void manipulationFeedbackCallbackExcavator(const move_excavator::ExcavationStatus::ConstPtr& msg);

  void setPoseGoalExcavator(move_base_msgs::MoveBaseGoal& poseGoal, double x, double y, double yaw); // m, m, rad
  void doneCallbackExcavator(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result);
  void activeCallbackExcavator();
  void feedbackCallbackExcavator(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);

  void RotateToHeadingExcavator(double desired_yaw);
  void UpdateGoalPoseExcavator();
  void ClearCostmapsExcavator();
  void LightsExcavator(std::string intensity);
  void RotateInPlaceExcavator(double throttle, double time);
  void StopExcavator(double time);
  void DriveExcavator(double throttle, double time);
  // void ToggleDetectorExcavator(bool flag);
  void BrakeExcavator(double intensity);
  void RoverStaticExcavator(bool flag);
  void DriveCmdVelExcavator(double vx, double vy, double wz, double time);
  void StartManipulation();
  void ExecuteHomeArmExcavator(double heading, double time);
  void ExecuteDigExcavator(double heading, double time);
  void ExecuteScoopExcavator(double heading, double time);
  void ExecuteExtendArmExcavator(double heading, double time);
  void ExecuteDropExcavator(double heading, double time);

  // HAULER
  void localizedBaseCallbackHauler(const std_msgs::Int64::ConstPtr& msg);
  void mobilityCallbackHauler(const std_msgs::Int64::ConstPtr& msg);
  void waypointUnreachableCallbackHauler(const std_msgs::Bool::ConstPtr& msg);
  void arrivedAtWaypointCallbackHauler(const std_msgs::Bool::ConstPtr& msg);
  void volatileDetectedCallbackHauler(const std_msgs::Float32::ConstPtr& msg);
  void volatileRecordedCallbackHauler(const std_msgs::Bool::ConstPtr& msg);
  void localizationFailureCallbackHauler(const std_msgs::Bool::ConstPtr& msg);
  void localizationCallbackHauler(const nav_msgs::Odometry::ConstPtr& msg);
  void drivingModeCallbackHauler(const std_msgs::Int64::ConstPtr& msg);
  void immobilityRecoveryHauler();
  void homingRecoveryHauler();

  void setPoseGoalHauler(move_base_msgs::MoveBaseGoal& poseGoal, double x, double y, double yaw); // m, m, rad
  void doneCallbackHauler(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result);
  void activeCallbackHauler();
  void feedbackCallbackHauler(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);

  void RotateToHeadingHauler(double desired_yaw);
  void UpdateGoalPoseHauler();
  void ClearCostmapsHauler();
  void LightsHauler(std::string intensity);
  void RotateInPlaceHauler(double throttle, double time);
  void StopHauler(double time);
  void DriveHauler(double throttle, double time);
  void ToggleDetectorHauler(bool flag);
  void BrakeHauler(double intensity);
  void RoverStaticHauler(bool flag);
  void DriveCmdVelHauler(double vx, double vy, double wz, double time);

};
