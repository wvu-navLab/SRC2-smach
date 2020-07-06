#include <ros/ros.h>
#include <vector>
#include <algorithm>
#include <std_msgs/Bool.h>

class SmRd1
{
public:
  // Members -------------------------------------------------------------------------------------------------------------------------
  const unsigned int num_states = 5;
  enum STATE_T {_initialize=0, _planning=1, _traverse=2, _volatile_handler=3, _lost=4};

  // Condition flag declarations
  bool flag_localized_base = false;          
  bool flag_have_true_pose = false;          
  bool flag_waypoint_unreachable = false;    
  bool flag_arrived_at_waypoint = true;      
  bool flag_volatile_detected = false;       
  bool flag_localizing_volatile = false;     
  bool flag_volatile_recorded = false;       
  bool flag_volatile_unreachable = false;    
  bool flag_localization_failure = false;    
  bool flag_recovering_localization = false;
  bool flag_brake_engaged = false;           
  bool flag_fallthrough_condition = false;   

  // State vector
  std::vector<int> state_to_exec; // Only one should be true at a time, if multiple are true then a default state should be executed

  // ROS objects
  ros::NodeHandle nh;
  ros::Subscriber localized_base_sub;
  ros::Subscriber waypoint_unreachable_sub;
  ros::Subscriber arrived_at_waypoint_sub;
  ros::Subscriber volatile_detected_sub;
  ros::Subscriber volatile_recorded_sub;
  ros::Subscriber localization_failure_sub;

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
  void localizedBaseCallback(const std_msgs::Bool::ConstPtr& msg);
  void waypointUnreachableCallback(const std_msgs::Bool::ConstPtr& msg);
  void arrivedAtWaypointCallback(const std_msgs::Bool::ConstPtr& msg);
  void volatileDetectedCallback(const std_msgs::Bool::ConstPtr& msg);
  void volatileRecordedCallback(const std_msgs::Bool::ConstPtr& msg);
  void localizationFailureCallback(const std_msgs::Bool::ConstPtr& msg);
};