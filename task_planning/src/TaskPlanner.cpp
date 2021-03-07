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
};



/////////////////////////////////////////////////////////////////////
/***************************CALLBACKS*******************************/
/////////////////////////////////////////////////////////////////////

void TaskPlanner::timeCallback(const rosgraph_msgs::Clock::ConstPtr &msg)
{

}

/**void TaskPlanner::volatileListCallback(const vol_data_type &msg)
{

}*/

void TaskPlanner::scout_pose_callback(const ros::MessageEvent<std_msgs::Bool const>& event)
{

  const ros::M_string& header = event.getConnectionHeader();
  std::string topic = header.at("topic");
  //ROS_WARN("HRMM %s",topic.c_str());
  char ind = topic.c_str()[SCOUT_STR_LOC];
  int index = std::atoi(&ind);
  const std_msgs::BoolConstPtr& msg = event.getMessage();

  ROS_WARN("%i",index);// << std::endl;
  //ROS_DEBUG("%d",msg->data);

}

void TaskPlanner::scout_monitor_callback(const ros::MessageEvent<std_msgs::String const>& event)
{
  const ros::M_string& header = event.getConnectionHeader();
  std::string topic = header.at("topic");
  int index = std::stoi(topic,&SCOUT_STR_LOC);
  const std_msgs::StringConstPtr& msg = event.getMessage();

}

void TaskPlanner::excavator_pose_callback(const ros::MessageEvent<std_msgs::String const>& event)
{
  const ros::M_string& header = event.getConnectionHeader();
  std::string topic = header.at("topic");
  int index = std::stoi(topic,&EXCAVATOR_STR_LOC);
  const std_msgs::StringConstPtr& msg = event.getMessage();

}

void TaskPlanner::excavator_monitor_callback(const ros::MessageEvent<std_msgs::String const>& event)
{
  const ros::M_string& header = event.getConnectionHeader();
  std::string topic = header.at("topic");
  int index = std::stoi(topic,&EXCAVATOR_STR_LOC);
  const std_msgs::StringConstPtr& msg = event.getMessage();

}

void TaskPlanner::hauler_pose_callback(const ros::MessageEvent<std_msgs::String const>& event)
{
  const ros::M_string& header = event.getConnectionHeader();
  std::string topic = header.at("topic");
  int index = std::stoi(topic,&HAULER_STR_LOC);
  const std_msgs::StringConstPtr& msg = event.getMessage();


}

void TaskPlanner::hauler_monitor_callback(const ros::MessageEvent<std_msgs::String const>& event)
{
  const ros::M_string& header = event.getConnectionHeader();
  std::string topic = header.at("topic");
  int index = std::stoi(topic,&HAULER_STR_LOC);
  const std_msgs::StringConstPtr& msg = event.getMessage();


}

/////////////////////////////////////////////////////////////////////
/***************************CONSTRUCTORS****************************/
/////////////////////////////////////////////////////////////////////

TaskPlanner::TaskPlanner(const CostFunction       & cost_function)
//,                   const std::vector<Robot> & robots)
{
  //cost_function_ = cost_function;
  //robots_ = robots;
  ROS_WARN("1");
  //setup robot subscribers
  int index_sub_scout = 1;
  int index_sub_excavator = 1;
  int index_sub_hauler = 1;

  int i = 0;
  //for (int i=0; i<robots.size(); i++) {
  //  switch(robots[i].type) {
  //    case SCOUT:
        std::string topic = "/small_scout_" + std::to_string(index_sub_scout) + "/localization/odometry/sensor_fusion";
        subs_robots_ = nh_.subscribe(topic, 10, &TaskPlanner::scout_pose_callback, this, ros::TransportHints().tcp());
        index_sub_scout++;
  /**      break;
      case EXCAVATOR:
        std::string topic = "/small_excavator_" + std::to_string(index_sub_excavator) + "/localization/odometry/sensor_fusion";
        subs_robots_[i] = nh_.subscribe(topic, 10, &TaskPlanner::excavator_callback, this);
        index_sub_excavator++;
        break;
      case HAULER:
        std::string topic = "/small_hauler_" + std::to_string(index_sub_hauler) + "/localization/odometry/sensor_fusion";
        subs_robots_[i] = nh_.subscribe(topic, 10, &TaskPlanner::hauler_callback, this);
        index_sub_hauler++;
        break;
      default:
        ROS_ERROR("TaskPlanner::TaskPlanner: robot type invalid!");
        break;
    }
  }*/

  //setup robot publishers
/**  int index_pub_scout = 1;
  int index_pub_excavator = 1;
  int index_pub_hauler = 1;
  for (int i=0; i<robots.size(); i++) {
    switch(robots[i].type) {
      case SCOUT:
        std::string topic = "/small_scout_" + std::to_string(index_pub_scout) + "/plan";
        pubs_plans_[i] = nh_.advertise<std_msgs::String>(topic, 10);
        index_pub_scout++;
        break;
      case EXCAVATOR:
        std::string topic = "/small_excavator_" + std::to_string(index_pub_excavator) + "/plan";
        pubs_plans_[i] = nh_.advertise<std_msgs::String>(topic, 10);
        index_pub_excavator++;
        break;
      case HAULER:
        std::string topic = "/small_hauler_" + std::to_string(index_pub_hauler) + "/plan";
        pubs_plans_[i] = nh_.advertise<std_msgs::String>(topic, 10);
        index_pub_hauler++;
        break;
      default:
        ROS_ERROR("TaskPlanner::TaskPlanner: robot type invalid!");
        break;
    }
  }*/

  //setup volatile subscribers
  //TODO
  std::cout << 2 << std::endl;
}

}
