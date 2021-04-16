// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file TaskPlanner.cpp
 */
//-----------------------------------------------------------------------------

#include <task_planning/TaskPlanner.hpp>

namespace mac {

//WARNING:
//planner sets volatiles forever

//TODO:
//maybe, change the publisher to a service that can be called by robots
//need a status topic from each robot, so the planner can know what the robots are doing

void TaskPlanner::
plan() const {
  int nearest_int, min_distance;
  std::vector<double> pose_min, vol_pose, default_pose, current_pose;
  default_pose.push_back(0);
  default_pose.push_back(0);
  vol_pose = default_pose;
  current_pose = default_pose;
  for (int i = 0; i < volatile_map_.vol.size(); ++i)
  {
    nearest_int = -1;
    min_distance = 5000;
    pose_min = default_pose;
    vol_pose[0] = volatile_map_.vol[i].position.point.x;
    vol_pose[1] = volatile_map_.vol[i].position.point.y;
    std::cout << "robots_.size() = " << robots_.size() << std::endl;
    for (int j = 0; j < robots_.size() ; ++j)
    {
        if (nearest_int != -1)
        {
          current_pose[0] = robots_[i].odom.pose.pose.position.x;
          current_pose[1] = robots_[i].odom.pose.pose.position.y;// robot of nearest in pose

          // robot of current in pose
          double distance = TaskPlanner::dist(current_pose, vol_pose);
          if (distance < min_distance)
          {
            min_distance = distance;
            nearest_int = j;
            pose_min = current_pose;
          }

        } else if (robots_[i].current_task < 0)
        {
          nearest_int = i;
          pose_min[0] = robots_[i].odom.pose.pose.position.x;
          pose_min[1] = robots_[i].odom.pose.pose.position.y;


        }
        if (nearest_int != -1){
          geometry_msgs::PointStamped msg;
          msg = volatile_map_.vol[nearest_int].position;
          pubs_plans_[nearest_int].publish(msg);
          std::cout << "publishing" << std::endl;
        }
    }

  }
  //do stuff
  //bool is_data_loaded = false;
  //while(is_data_loaded) {
    //check if data is loaded
  //  ros::spinOnce();
  //}

  //do planning

}


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
  std::string task_topic = "/task";
  for (int i=0; i<robots.size(); i++) {
    switch(robots[i].type) {
      case mac::SCOUT:
        topic = "/small_scout_" + std::to_string(index_pub_scout) + task_topic;
        pubs_plans_.push_back(nh_.advertise<geometry_msgs::PointStamped>(topic, 10));
        index_pub_scout++;
        break;
      case mac::EXCAVATOR:
        topic = "/small_excavator_" + std::to_string(index_pub_excavator) + task_topic;
        pubs_plans_.push_back(nh_.advertise<geometry_msgs::PointStamped>(topic, 10));
        index_pub_excavator++;
        break;
      case mac::HAULER:
        topic = "/small_hauler_" + std::to_string(index_pub_hauler) + task_topic;
        pubs_plans_.push_back(nh_.advertise<geometry_msgs::PointStamped>(topic, 10));
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

double TaskPlanner::dist(const std::vector<double> p1, const std::vector<double> p2) {
if(p1.size() != p2.size())
{
std::cout << "Error! p1.size() != p2.size() for computing distance!\n";
exit(1); //TODO: remove exit

double val=0;
for(int i=0; i<p1.size(); i++)
{
  double diff = p1[i] - p2[i];
  val = val + diff*diff;
}
val = std::sqrt(val);
return val;
}

}


}