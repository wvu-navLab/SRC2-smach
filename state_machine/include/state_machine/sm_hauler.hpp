// C++ headers
#include <vector>
#include <algorithm>
#include <boost/bind.hpp>

//ROS headers
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int64.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <srcp2_msgs/SpotLightSrv.h>
#include <srcp2_msgs/BrakeRoverSrv.h>
#include <srcp2_msgs/SystemPowerSaveSrv.h>
#include <srcp2_msgs/SystemMonitorMsg.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <waypoint_gen/GenerateWaypoint.h>
#include <waypoint_gen/StartWaypoint.h>
#include <waypoint_checker/CheckCollision.h>
#include <driving_tools/Stop.h>
#include <driving_tools/MoveForward.h>
#include <driving_tools/CirculateBaseStation.h>
#include <driving_tools/RotateInPlace.h>
#include <src2_approach_services/ApproachChargingStation.h>
#include <src2_approach_services/ApproachBin.h>
#include <src2_approach_services/FindBin.h>
#include <src2_approach_services/ApproachExcavator.h>
#include <range_to_base/LocationOfBin.h>
#include <range_to_base/LocationOfExcavator.h>
#include <sensor_fusion/RoverStatic.h>
#include <sensor_fusion/GetTruePose.h>
#include <sensor_fusion/ResetPosition.h>
#include <sensor_fusion/HomingUpdate.h>
#include <sensor_fusion/HomingUpdateProcessingPlant.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <task_planning/PlanInfo.h>
#include <task_planning/DumpCoordination.h>
#include <task_planning/Types.hpp>
#include <state_machine/RobotStatus.h>
#include <state_machine/HaulerStatus.h>
#include <state_machine/ExcavationStatus.h>
#include <waypoint_nav/GoToGoal.h>
#include <move_excavator/FindExcavator.h>
#include <src2_object_detection/FindObject.h>
#include <localization_watchdog/WatchdogStatus.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class SmHauler
{
public:
  // Members -------------------------------------------------------------------------------------------------------------------------
  const unsigned int num_states = 7;
  enum STATE_T {_initialize=0, _planning=1, _traverse=2, _volatile_handler=3, _lost=4, _emergency=5, _hauler_dumping=6};

  // State transition flag declarations
  bool flag_interrupt_plan = false;
  bool flag_have_true_pose = false;
  bool flag_spread_out = false;
  bool flag_emergency = false;
  bool flag_wasted = false;
  bool flag_arrived_at_waypoint = true;
  bool flag_localizing_volatile = false;
  bool flag_recovering_localization = false;
  bool flag_dumping = false;
  bool flag_brake_engaged = false;
  bool flag_fallthrough_condition = false;

  // Secondary flag declarations
  bool flag_immobile = false;
  bool flag_need_init_landmark = false;
  bool flag_localized_base = false;
  bool flag_full_bin = false;
  bool flag_approaching_side = false;
  bool flag_approached_side = false;
  bool flag_approaching_front = false;
  bool flag_approached_front = false;
  bool flag_approached_excavator = false;
  bool flag_located_excavator = false;
  bool flag_parked_hauler = false;
  bool flag_dumped = false;
  bool flag_called_get_true_pose = false;
  bool flag_first_volatile = true;
  bool flag_allowed_to_dump = false;

  ros::Time wp_checker_timer,laser_collision_timer, map_timer, waypoint_timer;

  // State vector
  std::vector<int> state_to_exec; // Only one should be true at a time, if multiple are true then a default state should be executed

  // ROS objects
  ros::NodeHandle nh;

  // Publishers
  ros::Publisher sm_status_pub;
  ros::Publisher cmd_vel_pub;
  ros::Publisher driving_mode_pub;
  ros::Publisher cmd_bin_pub;
  ros::Publisher hauler_status_pub;
  ros::Publisher cmd_sensor_yaw_pub;
  ros::Publisher cmd_sensor_pitch_pub;

  // Subscribers
  ros::Subscriber localized_base_sub;
  ros::Subscriber localization_sub;
  ros::Subscriber driving_mode_sub;
  ros::Subscriber laser_scan_sub;
  ros::Subscriber planner_interrupt_sub;
  ros::Subscriber init_attitude_sub; 
  ros::Subscriber system_monitor_sub;
  ros::Subscriber watchdog_sub; 
  std::vector<ros::Subscriber> excavator_odom_subs;
  std::vector<ros::Subscriber> excavation_status_subs;

  ros::ServiceClient clt_sf_true_pose;
  ros::ServiceClient clt_vh_report;
  ros::ServiceClient clt_stop;
  ros::ServiceClient clt_rip;
  ros::ServiceClient clt_drive;
  ros::ServiceClient clt_brake;
  ros::ServiceClient clt_lights;
  ros::ServiceClient clt_power;
  ros::ServiceClient clt_homing;
  ros::ServiceClient clt_homing_proc_plant;
  ros::ServiceClient clt_approach_base;
  ros::ServiceClient clt_approach_bin;
  ros::ServiceClient clt_find_bin;
  ros::ServiceClient clt_approach_excavator;
  ros::ServiceClient clt_rover_static;
  ros::ServiceClient clt_reset_position;
  ros::ServiceClient clt_waypoint_checker;
  ros::ServiceClient clt_srcp2_brake_rover;
  ros::ServiceClient clt_task_planning;
  ros::ServiceClient clt_location_of_bin;
  ros::ServiceClient clt_location_of_excavator;
  ros::ServiceClient clt_find_excavator;
  ros::ServiceClient clt_set_goal;
  ros::ServiceClient clt_go_to_goal;
  ros::ServiceClient clt_find_object;
  ros::ServiceClient clt_dump_coordination;

  MoveBaseClient ac;
  actionlib::SimpleClientGoalState move_base_state_;

  // Methods ----------------------------------------------------------------------------------------------------------------------------
  SmHauler(); // Constructor
  void run();
  // State methods
  void stateInitialize();
  void statePlanning();
  void stateTraverse();
  void stateVolatileHandler();
  void stateLost();
  void stateEmergency();
  void stateDump();

  /// Subscriber callbacks
  void localizedBaseCallback(const std_msgs::Int64::ConstPtr& msg);
  void systemMonitorCallback(const srcp2_msgs::SystemMonitorMsg::ConstPtr& msg);
  void localizationCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void drivingModeCallback(const std_msgs::Int64::ConstPtr& msg);
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void doneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result);
  void feedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
  void watchdogCallback(const localization_watchdog::WatchdogStatus::ConstPtr& msg);
  void activeCallback();
  void plannerInterruptCallback(const std_msgs::Bool::ConstPtr &msg);
  void initialAttitudeCallback(const geometry_msgs::QuaternionConstPtr& msg);
  void excavatorOdomCallback(const ros::MessageEvent<nav_msgs::Odometry const>& event);
  void excavationStatusCallback(const ros::MessageEvent<state_machine::ExcavationStatus const>& event);

  // Methods
  void CancelMoveBaseGoal();
  void SetMoveBaseGoal();
  void SetMoveBaseSpeed(double max_speed);
  void SetPoseGoal(move_base_msgs::MoveBaseGoal& poseGoal, double x, double y, double yaw); // m, m, rad
  void SetPowerMode(bool power_save);
  void ClearCostmaps(double wait_time);
  void Lights(double intensity);
  void GetTruePose(bool initialize_other_robot_attitude);
  void Drive(double speed_ratio, double time);
  void DriveCmdVel(double vx, double vy, double wz, double time);
  void CommandCamera(double yaw, double pitch, double time);
  void RotateToHeading(double desired_yaw);
  void RotateInPlace(double speed_ratio, double time);
  void Stop(double time);
  void Brake(double intensity);
  void BrakeRamp(double max_intensity, double time, int aggressivity);
  void RoverStatic(bool flag);
  void homingRecovery();
  void immobilityRecovery(int type);
  void CheckWaypoint(int max_count);
  bool GoToWaypoint(double distance_threshold, double y_offset_b);
  bool ApproachChargingStation(int max_count);
  bool ApproachExcavator(int max_count, double distance_threshold);
  bool ApproachBin(int max_count);
  bool HomingUpdate(bool init_landmark);
  bool HomingUpdateProcessingPlant();
  bool LocateBin();
  bool FindBin();
  void ExecuteShakeBin(double time);
  bool LocateExcavator();
  bool RequestDumping(bool dump_request);
  bool FindExcavator(double timeout);
  void PublishHaulerStatus();
  void CheckForCollision();
  void ResetPosition();
  void Plan();

  const int SCOUT_STR_LOC = 13; //index ~SHOULD BE~ at 14th position
  const int EXCAVATOR_STR_LOC = 17; //index ~SHOULD BE~ at 18th position
  const int HAULER_STR_LOC = 14; //index ~SHOULD BE~ at 15th position

  // Parameters
  std::string node_name_;
  std::string robot_name_;
  int robot_id_;
  int num_excavators_;

  double waypoint_type = 0;
  int driving_mode = 0;

  geometry_msgs::Pose current_pose_, goal_pose_;
  geometry_msgs::Point last_known_position_;
  geometry_msgs::Point base_location_;
  geometry_msgs::Point proc_plant_bin_location_;
  geometry_msgs::Point charging_station_location_;
  geometry_msgs::Point front_of_bin_location_;

  double x_proc_plant_ = -6;
  double y_proc_plant_ = 7;
  double x_repair_station_ = -6;
  double y_repair_station_ = -6;

  double power_rate_ = 0.0;
  double power_level_ = 100.0;

  // Transforms
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf2_listener;
  geometry_msgs::TransformStamped odom_to_base_footprint;
  geometry_msgs::TransformStamped base_footprint_to_odom;
  geometry_msgs::TransformStamped odom_to_arm_mount;
  geometry_msgs::TransformStamped camera_link_to_base_footprint;
  geometry_msgs::TransformStamped base_footprint_to_camera_link;
  geometry_msgs::TransformStamped camera_link_to_odom;
  geometry_msgs::TransformStamped odom_to_camera_link;

  geometry_msgs::PointStamped bucket_point_;

  double volatile_detected_distance = -1.0;
  double min_volatile_detected_distance = 30.0;
  double prev_volatile_detected_distance = -1.0;

  bool no_objective = false;

  int timer_counter = 0;

  double pitch_ = 0, roll_ = 0, yaw_ = 0, yaw_prev_ = 0;
  double goal_yaw_;

  int counter_laser_collision_ = 0;
  const double LASER_THRESH = 0.3;
  const int LASER_SET_SIZE = 20;
  const int LASER_COUNTER_THRESH = 20;

  double CRATER_RADIUS = 80.0;
  double HAULER_MAX_SPEED = 0.80;
  double curr_max_speed_= 0.80;

  int move_base_fail_counter = 0;

  int parking_recovery_counter_ = 0;
  int lost_recovery_counter_ = 0;

  // Planning
  task_planning::PlanInfo prev_srv_plan;

  // Excavation
  std::vector<nav_msgs::Odometry> small_excavators_odom_;
  std::vector<state_machine::ExcavationStatus> small_excavators_status_;

  int partner_excavator_id_;
  geometry_msgs::Point partner_excavator_location_;
  state_machine::ExcavationStatus partner_excavation_status_;

  int previous_scoop_counter = 0;
  double parking_left_offset = 0.0;
};
