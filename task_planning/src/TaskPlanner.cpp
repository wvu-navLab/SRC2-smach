// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file TaskPlanner.cpp
 */
//-----------------------------------------------------------------------------

#include <task_planning/TaskPlanner.hpp>

namespace mac {

void TaskPlanner::
plan() const {
  //do stuff
  bool is_data_loaded = false;
  while(is_data_loaded) {
    //check if data is loaded
    ros::spinOnce();
  }

  //do planning
};



/////////////////////////////////////////////////////////////////////
/***************************CALLBACKS*******************************/
/////////////////////////////////////////////////////////////////////

void TaskPlanner::timeCallback(const rosgraph_msgs::Clock::ConstPtr &msg)
{

}

/**void TaskPlanner::volatileListCallback(const vol_data_type &msg)
{
 // volatiles: point value of volatile, amount left?, position, type, uncertainty
}*/

void TaskPlanner::pose_callback(const ros::MessageEvent<std_msgs::Bool const>& event)
{

  const ros::M_string& header = event.getConnectionHeader();
  std::string topic = header.at("topic");
  char robot_type = topic.c_str()[7]; // first character is at 8th index

  const std_msgs::BoolConstPtr& msg = event.getMessage();

  char ind;
  int index;
  switch(robot_type){
    case 's': // scout
      ind = topic.c_str()[SCOUT_STR_LOC];
      index = std::atoi(&ind);

      break;
    case 'e': // excavator
      ind = topic.c_str()[SCOUT_STR_LOC];
      index = std::atoi(&ind);

      break;
    case 'h': // hauler
      ind = topic.c_str()[SCOUT_STR_LOC];
      index = std::atoi(&ind);

      break;
    default:
      ROS_ERROR("Incorrect Robot Type");

  }
  //ROS_WARN("HRMM %s",topic.c_str());
  ROS_WARN("%c %i",robot_type,index);// << std::endl;
  //ROS_DEBUG("%d",msg->data);

}

void TaskPlanner::monitor_callback(const ros::MessageEvent<std_msgs::Bool const>& event)
{
  const ros::M_string& header = event.getConnectionHeader();
  std::string topic = header.at("topic");
  char robot_type = topic.c_str()[7]; // first character is at 8th index

  const std_msgs::BoolConstPtr& msg = event.getMessage();

  char ind;
  int index;
  switch(robot_type){
    case 's': // scout
      ind = topic.c_str()[SCOUT_STR_LOC];
      index = std::atoi(&ind);

      break;
    case 'e': // excavator
      ind = topic.c_str()[SCOUT_STR_LOC];
      index = std::atoi(&ind);

      break;
    case 'h': // hauler
      ind = topic.c_str()[SCOUT_STR_LOC];
      index = std::atoi(&ind);

      break;
    default:
      ROS_ERROR("Incorrect Robot Type");

  }
  //ROS_WARN("HRMM %s",topic.c_str());
  ROS_WARN("%c %i",robot_type,index);// << std::endl;
  //ROS_DEBUG("%d",msg->data);

}

/////////////////////////////////////////////////////////////////////
/***************************CONSTRUCTORS****************************/
/////////////////////////////////////////////////////////////////////

TaskPlanner::TaskPlanner(const CostFunction       & cost_function,
                         const std::vector<Robot> & robots,
                         const PlanningParams &planning_params): cost_function_(cost_function), robots_(robots), planning_params_(planning_params)
{
  //setup robot subscribers
  int index_sub_scout = 1;
  int index_sub_excavator = 1;
  int index_sub_hauler = 1;

  std::string topic;

  std::string localization_topic = "/localization/odometry/sensor_fusion";
  for (int i=0; i<robots.size(); i++) {
    switch(robots[i].type) {
      case mac::SCOUT:
        topic = "/small_scout_" + std::to_string(index_sub_scout) + localization_topic;
        subs_robots_.push_back(nh_.subscribe(topic, 10, &TaskPlanner::pose_callback, this));
        index_sub_scout++;
        break;
      case mac::EXCAVATOR:
        topic = "/small_excavator_" + std::to_string(index_sub_excavator) + localization_topic;
        subs_robots_.push_back(nh_.subscribe(topic, 10, &TaskPlanner::pose_callback, this));
        index_sub_excavator++;
        break;
      case mac::HAULER:
        topic = "/small_hauler_" + std::to_string(index_sub_hauler) + localization_topic;
        subs_robots_.push_back(nh_.subscribe(topic, 10, &TaskPlanner::pose_callback, this));
        index_sub_hauler++;
        break;
      default:
        ROS_ERROR("TaskPlanner::TaskPlanner: robot type invalid!");
        break;
    }
  }

  //setup robot publishers
  int index_pub_scout = 1;
  int index_pub_excavator = 1;
  int index_pub_hauler = 1;
  std::string monitor_topic = "/system_monitor";
  for (int i=0; i<robots.size(); i++) {
    switch(robots[i].type) {
      case mac::SCOUT:
        topic = "/small_scout_" + std::to_string(index_pub_scout) + monitor_topic;
        pubs_plans_.push_back(nh_.advertise<std_msgs::String>(topic, 10));
        index_pub_scout++;
        break;
      case mac::EXCAVATOR:
        topic = "/small_excavator_" + std::to_string(index_pub_excavator) + monitor_topic;
        pubs_plans_.push_back(nh_.advertise<std_msgs::String>(topic, 10));
        index_pub_excavator++;
        break;
      case mac::HAULER:
        topic = "/small_hauler_" + std::to_string(index_pub_hauler) + monitor_topic;
        pubs_plans_.push_back(nh_.advertise<std_msgs::String>(topic, 10));
        index_pub_hauler++;
        break;
      default:
        ROS_ERROR("TaskPlanner::TaskPlanner: robot type invalid!");
        break;
    }
  }

  //setup volatile subscribers
  //TODO
}

}
