#include <ros/ros.h>
#include <vector>
#include <algorithm>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int64.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <waypoint_gen/GenerateWaypoint.h>
#include <waypoint_gen/StartWaypoint.h>
// #include <waypoint_nav/SetGoal.h>
// #include <waypoint_nav/Interrupt.h>
#include <driving_tools/Stop.h>
#include <driving_tools/MoveForward.h>
#include <driving_tools/CirculateBaseStation.h>
#include <driving_tools/RotateInPlace.h>
// #include <volatile_handler/VolatileReport.h>
// #include <volatile_handler/ToggleDetector.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <src2_object_detection/ApproachBaseStation.h>
#include <sensor_fusion/RoverStatic.h>
#include <sensor_fusion/GetTruePose.h>
#include <sensor_fusion/HomingUpdate.h>
#include <std_srvs/Empty.h>
#include <waypoint_checker/CheckCollision.h>
#include <boost/bind.hpp>
#include <state_machine/SetMobility.h>
#include <srcp2_msgs/SpotLightSrv.h>
#include <srcp2_msgs/BrakeRoverSrv.h>


#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <srcp2_msgs/ExcavatorScoopMsg.h>
#include <motion_control/ArmGroup.h>
#include <move_excavator/MultiAgentState.h>
#include <move_excavator/ExcavationStatus.h>
#include <move_excavator/HomeArm.h>
#include <move_excavator/LowerArm.h>
#include <move_excavator/Scoop.h>
#include <move_excavator/AfterScoop.h>
#include <move_excavator/ExtendArm.h>
#include <move_excavator/DropVolatile.h>
#include <move_excavator/ExcavatorFK.h>
#include <move_excavator/GoToPose.h>
#include <move_excavator/ControlInvJac.h>


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
#define LOWER_MODE 1
#define SCOOP_MODE 2
#define EXTEND_MODE 3
#define DROP_MODE 4
#define START 9

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class SmExcavator
{
public:
  // Members -------------------------------------------------------------------------------------------------------------------------
  const unsigned int num_states = 5;
  enum STATE_T {_initialize=0, _planning=1, _traverse=2, _volatile_handler=3, _lost=4};
 // State-machine mode
  int mode = HOME_MODE;
  actionlib::SimpleClientGoalState move_base_state_;
  
  // Condition flag declarations
  // bool flag_localized_base = false;
  int flag_localized_base = 0;
  int flag_mobility = 1;
  bool flag_have_true_pose = false;
  bool flag_waypoint_unreachable = false;
  bool flag_arrived_at_waypoint = true;
  bool flag_localizing_volatile = false;
  bool flag_volatile_recorded = false;
  bool flag_volatile_unreachable = false;
  bool flag_localization_failure = false;
  bool flag_recovering_localization = false;
  bool flag_brake_engaged = false;
  bool flag_fallthrough_condition = false;
  bool flag_completed_homing = false;
  bool flag_heading_fail=false;
  bool flag_manipulation_enabled_ = false;
  bool flag_bucket_full_ = false;
  bool flag_hauler_in_range_ = false;
  bool flag_found_volatile_ = false;
  bool flag_need_init_landmark=true;

  ros::Time manipulation_timer;
  ros::Time detection_timer, not_detected_timer, wp_checker_timer;
  ros::Time last_time_laser_collision, map_timer, waypoint_timer;

  int timer_counter = 0;
  double pitch_ = 0, roll_ = 0, yaw_ = 0, yaw_prev_ = 0;
  double goal_yaw_;
  bool actionDone_ = false;
  geometry_msgs::Pose current_pose_, goal_pose_;
  geometry_msgs::Point base_location_;
  double waypoint_type_;
  int driving_mode_;
  const double LASER_THRESH = 0.3;
  const int LASER_SET_SIZE = 20;
  const int LASER_COUNTER_THRESH = 20;

  int counter_laser_collision_ = 0;

  double volatile_detected_distance = -1.0;
  double min_volatile_detected_distance = 30.0;
  double prev_volatile_detected_distance = -1.0;

  // State vector
  std::vector<int> state_to_exec; // Only one should be true at a time, if multiple are true then a default state should be executed

  // ROS objects
  ros::NodeHandle nh;
  // Publishers
  ros::Publisher sm_state_pub;
  ros::Publisher cmd_vel_pub;
  ros::Publisher driving_mode_pub;
  ros::Publisher excavation_status_pub;
  // Subscribers
  ros::Subscriber localized_base_sub;
  ros::Subscriber waypoint_unreachable_sub;
  ros::Subscriber arrived_at_waypoint_sub;
  ros::Subscriber volatile_detected_sub;
  ros::Subscriber volatile_recorded_sub;
  ros::Subscriber localization_failure_sub;
  ros::Subscriber localization_sub;
  ros::Subscriber driving_mode_sub;
  ros::Subscriber laser_scan_sub;
  ros::Subscriber joint_states_sub;
  ros::Subscriber bucket_info_sub;
  ros::Subscriber goal_volatile_sub;
  ros::Subscriber target_bin_sub;
  ros::Subscriber manipulation_state_sub;
  // Clients
  ros::ServiceClient clt_sf_true_pose_;
  ros::ServiceClient clt_wp_gen_;
  ros::ServiceClient clt_wp_start_;
  ros::ServiceClient clt_vh_report_;
  ros::ServiceClient clt_stop_;
  ros::ServiceClient clt_rip_;
  ros::ServiceClient clt_drive_;
  ros::ServiceClient clt_brake_;
  ros::ServiceClient clt_lights_;
  ros::ServiceClient clt_homing_;
  ros::ServiceClient clt_approach_base_;
  ros::ServiceClient clt_rover_static_;
  ros::ServiceClient clt_waypoint_checker_;
  ros::ServiceClient clt_srcp2_brake_rover_;
  ros::ServiceClient clt_home_arm;
  ros::ServiceClient clt_extend_arm;
  ros::ServiceClient clt_lower_arm;
  ros::ServiceClient clt_scoop;
  ros::ServiceClient clt_after_scoop;
  ros::ServiceClient clt_drop_volatile;
  ros::ServiceClient clt_forward_kin;
  ros::ServiceClient clt_go_to_pose;
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
  void volatileDetectedCallback(const std_msgs::Float32::ConstPtr& msg);
  void volatileRecordedCallback(const std_msgs::Bool::ConstPtr& msg);
  void localizationFailureCallback(const std_msgs::Bool::ConstPtr& msg);
  void localizationCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void drivingModeCallback(const std_msgs::Int64::ConstPtr& msg);
  // Callback function for excavation state machines.
  void bucketCallback(const srcp2_msgs::ExcavatorScoopMsg::ConstPtr &msg);
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void goalVolatileCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void targetBinCallback(const geometry_msgs::PointStamped::ConstPtr &msg);
  void manipulationStateCallback(const std_msgs::Int64::ConstPtr &msg);
  void doneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result);
  void activeCallback();
  void feedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);

  void immobilityRecovery(int type);
  void homingRecovery();

  void setPoseGoal(move_base_msgs::MoveBaseGoal& poseGoal, double x, double y, double yaw); // m, m, rad

  void RotateToHeading(double desired_yaw);
  void ClearCostmaps();
  void Lights(double intensity);
  void RotateInPlace(double throttle, double time);
  void Stop(double time);
  void Drive(double throttle, double time);
  // void ToggleDetector(bool flag);
  void Brake(double intensity);
  void RoverStatic(bool flag);
  void DriveCmdVel(double vx, double vy, double wz, double time);
  void BrakeRamp(double max_intensity, double time, int aggressivity);
  
  bool setMobility(state_machine::SetMobility::Request &req, state_machine::SetMobility::Response &res);

  void executeHomeArm(double timeout);
  void executeLowerArm(double timeout);
  void executeScoop(double timeout);
  void executeAfterScoop(double timeout);
  void executeExtendArm(double timeout);
  void executeDrop(double timeout);
  void executeGoToPose(double timeout, const geometry_msgs::PoseStamped::ConstPtr &msg);
  void outputManipulationStatus();
  void getRelativePosition();
  void getForwardKinematics(double timeout);

  // Parameters
  std::string robot_name_;
  std::string node_name_;

  // Joint Positions Init
  double q1_pos_ = 0.0;
  double q2_pos_ = 0.0;
  double q3_pos_ = 0.0;
  double q4_pos_ = 0.0;

  // Bucket Info Init
  bool volatile_in_bucket_ = false;
  bool regolith_in_bucket = false;

  // End-effector Pose Init
  geometry_msgs::PoseStamped eePose_;
  geometry_msgs::PoseStamped volatile_pose_;
  double volatile_heading_ = 0;
  geometry_msgs::PointStamped bin_point_;
  double relative_heading_ = 1.57;

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

};
