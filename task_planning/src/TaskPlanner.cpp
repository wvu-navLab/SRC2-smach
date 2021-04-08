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
  time_ = msg->clock;
}

void TaskPlanner::volatileMapCallback(const volatile_map::VolatileMap::ConstPtr &msg)
{
 volatile_map_ = *msg;
}

void TaskPlanner::poseCallback(const ros::MessageEvent<nav_msgs::Odometry const>& event)
{

  const ros::M_string& header = event.getConnectionHeader();
  std::string topic = header.at("topic");
  char robot_type = topic.c_str()[7]; // first character is at 8th index

  const nav_msgs::OdometryConstPtr& msg = event.getMessage();

  char ind;
  int id;
  switch(robot_type){
    case 's': // scout
      ind = topic.c_str()[SCOUT_STR_LOC];
      id = std::atoi(&ind);

      break;
    case 'e': // excavator
      ind = topic.c_str()[EXCAVATOR_STR_LOC];
      id = std::atoi(&ind);

      break;
    case 'h': // hauler
      ind = topic.c_str()[HAULER_STR_LOC];
      id = std::atoi(&ind);

      break;
    default:
      ROS_ERROR("Incorrect Robot Type");

  }
  //ROS_WARN("HRMM %s",topic.c_str());
  ROS_WARN("%c %i",robot_type,id);// << std::endl;
  //ROS_DEBUG("%d",msg->data);
  int index = getRobotIndex(robot_type, id);
  robots_[index].odom = *msg;

}

/**void TaskPlanner::taskStatusCallback(const ros::MessageEvent<std_msgs::Bool const>& event)
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

}*/

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
  std::string localization_topic;
  if (planning_params.demo){
    localization_topic = "/localization/odometry/truth";
  } else
  {
    localization_topic = "/localization/odometry/sensor_fusion";
  }


  for (int i=0; i<robots.size(); i++) {
    switch(robots[i].type) {
      case mac::SCOUT:
        topic = "/small_scout_" + std::to_string(index_sub_scout) + localization_topic;
        subs_robots_.push_back(nh_.subscribe(topic, 10, &TaskPlanner::poseCallback, this));
        index_sub_scout++;
        break;
      case mac::EXCAVATOR:
        topic = "/small_excavator_" + std::to_string(index_sub_excavator) + localization_topic;
        subs_robots_.push_back(nh_.subscribe(topic, 10, &TaskPlanner::poseCallback, this));
        index_sub_excavator++;
        break;
      case mac::HAULER:
        topic = "/small_hauler_" + std::to_string(index_sub_hauler) + localization_topic;
        subs_robots_.push_back(nh_.subscribe(topic, 10, &TaskPlanner::poseCallback, this));
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

  sub_clock_ = nh_.subscribe("/clock", 10, &TaskPlanner::timeCallback, this);
  sub_volatiles_ = nh_.subscribe("/volatile_map", 10, &TaskPlanner::volatileMapCallback, this);

  //setup volatile subscribers
  //TODO
}

/////////////////////////////////////////////////////////////////////
/***************************UTILITIES*******************************/
/////////////////////////////////////////////////////////////////////

int TaskPlanner::getRobotIndex(char robot_type, int robot_id)
{
  int index = -1;
  for (int i = 0; i < robots_.size(); ++i)
  {
    if (robots_[i].type == robot_type && robots_[i].id == robot_id)
    {
      index = i;
    }
  }
  return index;
}

}
