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


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class SmRd1
{
public:
  // Members -------------------------------------------------------------------------------------------------------------------------
  const unsigned int num_states = 5;
  enum STATE_T {_initialize=0, _planning=1, _traverse=2, _volatile_handler=3, _lost=4};

  // Condition flag declarations
  // bool flag_localized_base = false;
  int flag_localized_base = 0;
  int flag_mobility = 1;
  bool flag_have_true_pose = false;
  bool flag_waypoint_unreachable = false;
  bool flag_arrived_at_waypoint = true;
  double volatile_detected_distance = -1.0;
  double min_volatile_detected_distance = 30.0;
  double prev_volatile_detected_distance = -1.0;
  bool flag_localizing_volatile = false;
  bool flag_volatile_recorded = false;
  bool flag_volatile_unreachable = false;
  bool flag_localization_failure = false;
  bool flag_recovering_localization = false;
  bool flag_brake_engaged = false;
  bool flag_fallthrough_condition = false;
  bool flag_completed_homing = false;
  bool flag_heading_fail=false;
  bool need_to_initialize_landmark=true;

  ros::Time detection_timer, not_detected_timer, wp_checker_timer;
  ros::Time last_time_laser_collision_, map_timer, waypoint_timer_;

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


  // State vector
  std::vector<int> state_to_exec; // Only one should be true at a time, if multiple are true then a default state should be executed

  // ROS objects
  ros::NodeHandle nh;
  ros::Publisher sm_state_pub, cmd_vel_pub, pub_driving_mode_;
  ros::Subscriber localized_base_sub;
  ros::Subscriber waypoint_unreachable_sub;
  ros::Subscriber arrived_at_waypoint_sub;
  ros::Subscriber volatile_detected_sub;
  ros::Subscriber volatile_recorded_sub;
  ros::Subscriber localization_failure_sub;
  ros::Subscriber localization_sub;
  ros::Subscriber driving_mode_sub;
  // ros::Subscriber mobility_sub;
  ros::Subscriber laserscan_sub;

  // ros::ServiceClient clt_true_pose_;
  ros::ServiceClient clt_sf_true_pose_;
  ros::ServiceClient clt_wp_gen_;
  ros::ServiceClient clt_wp_start_;
  // ros::ServiceClient clt_wp_nav_set_goal_;
  // ros::ServiceClient clt_wp_nav_interrupt_;
  ros::ServiceClient clt_vh_report_;
  ros::ServiceClient clt_stop_;
  ros::ServiceClient clt_rip_;
  ros::ServiceClient clt_drive_;
  // ros::ServiceClient clt_vol_report_;
  // ros::ServiceClient clt_vol_detect_;
  ros::ServiceClient clt_brake_;
  ros::ServiceClient clt_lights_;
  ros::ServiceClient clt_homing_;
  ros::ServiceClient clt_approach_base_;
  ros::ServiceClient clt_rover_static_;
  ros::ServiceClient clt_waypoint_checker_;
  ros::ServiceClient clt_srcp2_brake_rover_;

  ros::ServiceServer setMobilityService_;







  MoveBaseClient ac;
  actionlib::SimpleClientGoalState move_base_state_;

  // Methods ----------------------------------------------------------------------------------------------------------------------------
  SmRd1(); // Constructor
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
  void immobilityRecovery(int type);
  void homingRecovery();
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

  void setPoseGoal(move_base_msgs::MoveBaseGoal& poseGoal, double x, double y, double yaw); // m, m, rad
  void doneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result);
  void activeCallback();
  void feedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
  bool setMobility_(state_machine::SetMobility::Request &req, state_machine::SetMobility::Response &res);

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

};
