// C++ headers
#include <vector>
#include <algorithm>
#include <boost/bind.hpp>

// ROS headers
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/JointState.h>
#include <srcp2_msgs/SpotLightSrv.h>
#include <srcp2_msgs/BrakeRoverSrv.h>
#include <srcp2_msgs/ExcavatorScoopMsg.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <waypoint_gen/GenerateWaypoint.h>
#include <waypoint_gen/StartWaypoint.h>
#include <waypoint_checker/CheckCollision.h>
#include <driving_tools/Stop.h>
#include <driving_tools/MoveForward.h>
#include <driving_tools/MoveSideways.h>
#include <driving_tools/TurnWheelsSideways.h>
#include <driving_tools/CirculateBaseStation.h>
#include <driving_tools/RotateInPlace.h>
#include <src2_approach_services/ApproachChargingStation.h>
#include <sensor_fusion/RoverStatic.h>
#include <sensor_fusion/GetTruePose.h>
#include <sensor_fusion/HomingUpdate.h>
#include <state_machine/SetMobility.h>
#include <actionlib/client/simple_action_client.h>
#include <motion_control/ArmGroup.h>
#include <move_excavator/HomeArm.h>
#include <move_excavator/LowerArm.h>
#include <move_excavator/Scoop.h>
#include <move_excavator/AfterScoop.h>
#include <move_excavator/ExtendArm.h>
#include <move_excavator/DropVolatile.h>
#include <move_excavator/ExcavatorFK.h>
#include <move_excavator/GoToPose.h>
#include <move_excavator/ControlInvJac.h>
#include <move_excavator/FindHauler.h>
#include <task_planning/PlanInfo.h>
#include <task_planning/Types.hpp>
#include <state_machine/RobotStatus.h>
#include <state_machine/ExcavationStatus.h>


#define PI 3.141592653589793

#define JOINT1_MAX PI
#define JOINT1_MIN -PI
#define JOINT2_MAX PI/3
#define JOINT2_MIN -PI/5
#define JOINT3_MAX PI/3
#define JOINT3_MIN -PI/3
#define JOINT4_MAX 5*PI/4
#define JOINT4_MIN 0

#define STOP -1
#define HOME_MODE 0
#define SEARCH_MODE 1
#define LOWER_MODE 2
#define SCOOP_MODE 3
#define EXTEND_MODE 4
#define DROP_MODE 5
#define START 9

#define EXCAVATION_ENABLED 0
#define EXCAVATION_READY_TO_DUMP 1

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class SmExcavator
{
public:
  // Members -------------------------------------------------------------------------------------------------------------------------
  const unsigned int num_states = 5;
  enum STATE_T {_initialize=0, _planning=1, _traverse=2, _volatile_handler=3, _lost=4};
 // State-machine mode
  int excavation_state = HOME_MODE;
  actionlib::SimpleClientGoalState move_base_state;

  // State transition flag declarations
  bool flag_interrupt_plan = false;
  bool flag_have_true_pose = false;
  bool flag_arrived_at_waypoint = true;
  bool flag_localizing_volatile = false;
  bool flag_recovering_localization = false;
  bool flag_brake_engaged = false;
  bool flag_fallthrough_condition = false;

  // Secondary flag declarations
  bool flag_waypoint_unreachable = false;
  bool flag_localization_failure = false;
  bool flag_completed_homing = false;
  bool flag_heading_fail=false;
  bool flag_manipulation_enabled = false;
  bool flag_bucket_full = false;
  bool flag_hauler_in_range = false;
  bool flag_found_volatile = false;
  bool flag_need_init_landmark = false;
  bool flag_volatile_dug=true;
  bool flag_localized_base = false;
  bool flag_mobility = true;

  ros::Time manipulation_timer;
  ros::Time detection_timer, not_detected_timer, wp_checker_timer;
  ros::Time laser_collision_timer, map_timer, waypoint_timer;

  // State vector
  std::vector<int> state_to_exec; // Only one should be true at a time, if multiple are true then a default state should be executed

  // ROS objects
  ros::NodeHandle nh;
  // Publishers
  ros::Publisher sm_status_pub;
  ros::Publisher cmd_vel_pub;
  ros::Publisher driving_mode_pub;
  ros::Publisher excavation_status_pub;

  // Subscribers
  ros::Subscriber localized_base_sub;
  ros::Subscriber waypoint_unreachable_sub;
  ros::Subscriber arrived_at_waypoint_sub;
  ros::Subscriber localization_failure_sub;
  ros::Subscriber localization_sub;
  ros::Subscriber driving_mode_sub;
  ros::Subscriber laser_scan_sub;
  ros::Subscriber joint_states_sub;
  ros::Subscriber bucket_info_sub;
  ros::Subscriber goal_volatile_sub;
  ros::Subscriber target_bin_sub;
  ros::Subscriber manipulation_cmd_sub;
  ros::Subscriber planner_interrupt_sub;

  // Services
  ros::ServiceClient clt_sf_true_pose;
  ros::ServiceClient clt_wp_gen;
  ros::ServiceClient clt_wp_start;
  ros::ServiceClient clt_vh_report;
  ros::ServiceClient clt_stop;
  ros::ServiceClient clt_rip;
  ros::ServiceClient clt_move_side;
  ros::ServiceClient clt_turn_wheels_side;
  ros::ServiceClient clt_drive;
  ros::ServiceClient clt_brake;
  ros::ServiceClient clt_lights;
  ros::ServiceClient clt_homing;
  ros::ServiceClient clt_approach_base;
  ros::ServiceClient clt_rover_static;
  ros::ServiceClient clt_waypoint_checker;
  ros::ServiceClient clt_srcp2_brake_rover;
  ros::ServiceClient clt_home_arm;
  ros::ServiceClient clt_extend_arm;
  ros::ServiceClient clt_lower_arm;
  ros::ServiceClient clt_scoop;
  ros::ServiceClient clt_after_scoop;
  ros::ServiceClient clt_drop_volatile;
  ros::ServiceClient clt_forward_kin;
  ros::ServiceClient clt_go_to_pose;
  ros::ServiceClient clt_find_hauler;
  ros::ServiceClient clt_task_planning;

  MoveBaseClient ac;

  // Clients
  ros::ServiceServer srv_mobility;

  // Methods ----------------------------------------------------------------------------------------------------------------------------
  SmExcavator(); // Constructor
  void run();

  // State methods
  void stateInitialize();
  void statePlanning();
  void stateTraverse();
  void stateVolatileHandler();
  void stateLost();

  // Subscriber callbacks
  void localizedBaseCallback(const std_msgs::Int64::ConstPtr& msg);
  void mobilityCallback(const std_msgs::Int64::ConstPtr& msg);
  void waypointUnreachableCallback(const std_msgs::Bool::ConstPtr& msg);
  void arrivedAtWaypointCallback(const std_msgs::Bool::ConstPtr& msg);
  void localizationFailureCallback(const std_msgs::Bool::ConstPtr& msg);
  void localizationCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void drivingModeCallback(const std_msgs::Int64::ConstPtr& msg);
  void bucketCallback(const srcp2_msgs::ExcavatorScoopMsg::ConstPtr &msg);
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void goalVolatileCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void targetBinCallback(const geometry_msgs::PointStamped::ConstPtr &msg);
  void manipulationCmdCallback(const std_msgs::Int64::ConstPtr &msg);
  void activeCallback();
  void feedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
  void doneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result);
  void plannerInterruptCallback(const std_msgs::Bool::ConstPtr &msg);

  // Service providers
  bool setMobility(state_machine::SetMobility::Request &req, state_machine::SetMobility::Response &res);

  // Methods
  void setPoseGoal(move_base_msgs::MoveBaseGoal& poseGoal, double x, double y, double yaw); // m, m, rad
  void ClearCostmaps();
  void Lights(double intensity);
  void GetTruePose();
  void Drive(double speed_ratio, double time);
  void DriveCmdVel(double vx, double vy, double wz, double time);
  void RotateToHeading(double desired_yaw);
  void RotateInPlace(double speed_ratio, double time);
  void MoveSideways(double speed_ratio, double time);
  void TurnWheelsSideways(bool start, double time);
  void Stop(double time);
  void Brake(double intensity);
  void BrakeRamp(double max_intensity, double time, int aggressivity);
  void RoverStatic(bool flag);
  void homingRecovery();
  void immobilityRecovery(int type);
  void CheckWaypoint(int max_count);
  bool ApproachChargingStation(int max_count);
  void FindHauler(double timeout);
  void ExcavationStateMachine();
  void ExecuteHomeArm(double timeout);
  void ExecuteLowerArm(double timeout);
  void ExecuteScoop(double timeout);
  void ExecuteAfterScoop(double timeout);
  void ExecuteExtendArm(double timeout);
  void ExecuteDrop(double timeout);
  void ExecuteGoToPose(double timeout, const geometry_msgs::PointStamped &point);
  void getForwardKinematics(double timeout);
  void getRelativePosition();
  void PublishExcavationStatus();
  bool HomingUpdate(bool init_landmark);
  void Plan();

  // Parameters
  std::string node_name_;
  std::string robot_name_;
  int robot_id_;

  double waypoint_type;
  int driving_mode;

  // Bucket Info Init
  bool volatile_in_bucket = false;
  bool regolith_in_bucket = false;
  bool no_objective = false;

  // End-effector Pose Init
  geometry_msgs::Pose current_pose_;
  geometry_msgs::Pose goal_pose_;
  geometry_msgs::PoseStamped eePose_;
  geometry_msgs::PoseStamped volatile_pose_;
  geometry_msgs::Point base_location_;
  geometry_msgs::PointStamped bin_point_;

  // Transforms
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf2_listener;
  geometry_msgs::TransformStamped odom_to_base_link;
  geometry_msgs::TransformStamped base_link_to_odom;
  geometry_msgs::TransformStamped base_link_to_arm_mount;
  geometry_msgs::TransformStamped arm_mount_to_base_link;
  geometry_msgs::TransformStamped odom_to_arm_mount;
  geometry_msgs::TransformStamped arm_mount_to_odom;
  geometry_msgs::TransformStamped camera_link_to_arm_mount;
  geometry_msgs::TransformStamped arm_mount_to_camera_link;

  double volatile_heading_ = 0;
  double relative_heading_ = 1.57;
  double pitch_ = 0, roll_ = 0, yaw_ = 0, yaw_prev_ = 0;
  double goal_yaw_;

  bool actionDone_ = false;

  int timer_counter = 0;

  int counter_laser_collision_ = 0;
  const double LASER_THRESH = 0.3;
  const int LASER_SET_SIZE = 20;
  const int LASER_COUNTER_THRESH = 20;

  // Joint Positions Init
  double q1_pos_ = 0.0;
  double q2_pos_ = 0.0;
  double q3_pos_ = 0.0;
  double q4_pos_ = 0.0;

  // Planning
};
