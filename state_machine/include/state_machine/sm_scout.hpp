#include <ros/ros.h>
#include <vector>
#include <algorithm>

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
#include <srcp2_msgs/VolSensorMsg.h>
#include <srcp2_msgs/ExcavatorScoopMsg.h>
#include <srcp2_msgs/SystemMonitorMsg.h>
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
#include <actionlib/client/simple_action_client.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <task_planning/PlanInfo.h>
#include <task_planning/Types.hpp>
#include <state_machine/RobotStatus.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class SmScout
{
public:
  // Members -------------------------------------------------------------------------------------------------------------------------
  const unsigned int num_states = 5;
  enum STATE_T {_initialize=0, _planning=1, _traverse=2, _volatile_handler=3, _lost=4};

  // State transition flag declarations
  bool flag_interrupt_plan = false;
  bool flag_have_true_pose = false;
  bool flag_arrived_at_waypoint = true;
  bool flag_localizing_volatile = false;
  bool flag_recovering_localization = false;
  bool flag_brake_engaged = false;
  bool flag_fallthrough_condition = false;

  // Secondary flag declarations
  bool flag_need_init_landmark = false;
  bool flag_localized_base = false;
  bool flag_volatile_honed = false;
  bool flag_volatile_unreachable = false;
  bool flag_completed_homing = false;
  bool flag_heading_fail=false;
  bool flag_volatile_detected = false;

  ros::Time wp_checker_timer, laser_collision_timer, map_timer, waypoint_timer;

  // State vector
  std::vector<int> state_to_exec; // Only one should be true at a time, if multiple are true then a default state should be executed

  // ROS objects
  ros::NodeHandle nh;
  // Publishers
  ros::Publisher sm_status_pub;
  ros::Publisher cmd_vel_pub;
  ros::Publisher driving_mode_pub;

  // Subscribers
  ros::Subscriber localized_base_sub;
  ros::Subscriber volatile_sensor_sub;
  ros::Subscriber volatile_cmd_sub;
  ros::Subscriber localization_sub;
  ros::Subscriber driving_mode_sub;
  ros::Subscriber laser_scan_sub;
  ros::Subscriber planner_interrupt_sub;
  ros::Subscriber system_monitor_sub;
  // Services
  ros::ServiceClient clt_sf_true_pose;
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
  ros::ServiceClient clt_task_planning;
  MoveBaseClient ac;

  // Clients
  ros::ServiceServer srv_mobility;

  actionlib::SimpleClientGoalState move_base_state;

  // Methods ----------------------------------------------------------------------------------------------------------------------------
  SmScout(); // Constructor
  void run();
  // State methods
  void stateInitialize();
  void statePlanning();
  void stateTraverse();
  void stateVolatileHandler();
  void stateLost();

  /// Subscriber callbacks
  void localizedBaseCallback(const std_msgs::Int64::ConstPtr& msg);
  void localizationCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void drivingModeCallback(const std_msgs::Int64::ConstPtr& msg);
  void volatileCmdCallback(const std_msgs::Int64::ConstPtr& msg);
  void volatileSensorCallback(const srcp2_msgs::VolSensorMsg::ConstPtr& msg);
  void systemMonitorCallback(const srcp2_msgs::SystemMonitorMsg::ConstPtr& msg);
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void doneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result);
  void feedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
  void activeCallback();
  void plannerInterruptCallback(const std_msgs::Bool::ConstPtr &msg);

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
  bool HomingUpdate(bool init_landmark);
  void Plan();

  // Parameters
  std::string node_name_;
  std::string robot_name_;
  int robot_id_;

  double waypoint_type_ = 0;
  int driving_mode_ = 0;

  geometry_msgs::Pose current_pose_, goal_pose_;
  geometry_msgs::Point base_location_;

  double vol_detected_dist_ = -1.0;
  double min_vol_detected_dist_ = 30.0;
  double prev_vol_detected_dist_ = -1.0;
  double power_rate_;
  double power_level_;


  bool no_objective =false;
  int timer_counter = 0;

  double pitch_ = 0, roll_ = 0, yaw_ = 0, yaw_prev_ = 0;
  double goal_yaw_;

  int counter_laser_collision_ = 0;
  const double LASER_THRESH = 0.3;
  const int LASER_SET_SIZE = 20;
  const int LASER_COUNTER_THRESH = 20;

  const double VOL_FOUND_THRESH = 0.3;
  int honing_direction_ = 1;

  // Planning
  task_planning::PlanInfo prev_srv_plan;
};
