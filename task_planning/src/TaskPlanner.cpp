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

//add call help message and accept/denial so excavators can get scout if need be
// lazy excavator/hauler following


/////////////////////////////////////////////////////////////////////
/***************************PLANNERS********************************/
/////////////////////////////////////////////////////////////////////

void TaskPlanner::populate_prior_plan()
{
  if (!planning_params_.plan.empty()){
    for(int i = 0; i < planning_params_.plan.size(); ++i)
    {
      geometry_msgs::PointStamped temp;
      for (int j = 0; i < robots_.size(); ++j)
      {
        if (robots_[j].type == planning_params_.plan[0][i] && robots_[j].id == planning_params_.plan[1][i])
        {
          temp.point.x = planning_params_.plan[2][i];
          temp.point.y = planning_params_.plan[3][i];
          robots_[j].plan.push_back(temp);
          ROS_INFO_STREAM("Plan: " << temp.point.x << ", " << temp.point.y);
        }
      }
    }
  }
}

void TaskPlanner::scout_plan_default(int type, int id)
{
  for (auto&robot: robots_)
  {
    if (type == robot.type && id == robot.id)
    {
      ROS_INFO_STREAM("Plan " << robot.plan[0]);
      robot.plan.erase(robot.plan.begin());
      ROS_INFO_STREAM("Plan " << robot.plan[0]);
    }
  }
}

void TaskPlanner::exc_haul_plan_default()
{
  for (auto&robot: robots_)
  {
    robot.plan.clear();
  }

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
    ROS_ERROR_STREAM("Volatile pose " << vol_pose[0] << "," << vol_pose[1]);
    // std::cout << "robots_.size() = " << robots_.size() << std::endl;
    for (int j = 0; j < robots_.size() ; ++j)
    {
      if (robots_[j].type == mac::EXCAVATOR)
      {
        if (nearest_int != -1)
        {
          current_pose[0] = robots_[j].odom.pose.pose.position.x;
          current_pose[1] = robots_[j].odom.pose.pose.position.y;// robot of nearest in pose
          ROS_ERROR_STREAM("Excavator pose " << current_pose[0] << "," << current_pose[1]);

          // robot of current in pose
          double distance = TaskPlanner::dist(current_pose, vol_pose);
          if (distance < min_distance)
          {
            min_distance = distance;
            nearest_int = j;
            pose_min = current_pose;
          }

        } else if (robots_[j].current_task < 0)
        {
          nearest_int = j;
          pose_min[0] = robots_[j].odom.pose.pose.position.x;
          pose_min[1] = robots_[j].odom.pose.pose.position.y;
          // ROS_ERROR_STREAM("Excavator pose min " << pose_min[0] << "," << pose_min[1]);
        }
      }
    }
    geometry_msgs::PointStamped temp;
    temp.point.x = vol_pose[0];
    temp.point.y = vol_pose[1];
    robots_[nearest_int].plan.push_back(temp);
    nearest_int = -1;
    min_distance = 5000;
    pose_min = default_pose;
    vol_pose[0] = volatile_map_.vol[i].position.point.x;
    vol_pose[1] = volatile_map_.vol[i].position.point.y;
    // std::cout << "robots_.size() = " << robots_.size() << std::endl;
    for (int j = 0; j < robots_.size() ; ++j)
    {
      if (robots_[j].type == mac::HAULER)
      {
        if (nearest_int != -1)
        {
          current_pose[0] = robots_[j].odom.pose.pose.position.x;
          current_pose[1] = robots_[j].odom.pose.pose.position.y;// robot of nearest in pose
          ROS_ERROR_STREAM("Hauler pose " << current_pose[0] << "," << current_pose[1]);

          // robot of current in pose
          double distance = TaskPlanner::dist(current_pose, vol_pose);
          if (distance < min_distance)
          {
            min_distance = distance;
            nearest_int = j;
            pose_min = current_pose;
          }

        } else if (robots_[j].current_task < 0)
        {
          nearest_int = j;
          pose_min[0] = robots_[j].odom.pose.pose.position.x;
          pose_min[1] = robots_[j].odom.pose.pose.position.y;
        }
      }
    }
    robots_[nearest_int].plan.push_back(temp);
  }
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

  int r_type;

  const nav_msgs::OdometryConstPtr& msg = event.getMessage();

  char ind;
  int id;
  switch(robot_type){
    case 's': // scout
      ind = topic.c_str()[SCOUT_STR_LOC];
      id = std::atoi(&ind);
      r_type = mac::SCOUT;

      break;
    case 'e': // excavator
      ind = topic.c_str()[EXCAVATOR_STR_LOC];
      id = std::atoi(&ind);
      r_type = mac::EXCAVATOR;

      break;
    case 'h': // hauler
      ind = topic.c_str()[HAULER_STR_LOC];
      id = std::atoi(&ind);
      r_type = mac::HAULER;

      break;
    default:
      ROS_ERROR("Incorrect Robot Type");

  }
  //ROS_WARN("HRMM %s",topic.c_str());
  // ROS_WARN("%c %i",robot_type,id);// << std::endl;
  //ROS_DEBUG("%d",msg->data);
  int index = get_robot_index(r_type, id);
  // ROS_ERROR_STREAM("Index "<<index);

  // ROS_ERROR_STREAM("Robot pose "<<*msg);
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

bool TaskPlanner::taskPlanService(task_planning::PlanInfo::Request &req,task_planning::PlanInfo::Response &res)
{

  ROS_ERROR("Planning started.");
    if (req.replan.data)
    {
      //perform planning type
      switch(planning_params_.type)
      {
        case SCOUT_PLANNER_DEFAULT :
          this->scout_plan_default(req.type.data, req.id.data);
          break;
        case EXC_HAUL_PLANNER_DEFAULT :
          this->exc_haul_plan_default();
          break;

        default:
          ROS_ERROR("Task Planner type invalid!");
          break;

      }
      std_msgs::Bool msg;
      msg.data = true;
      pub_interrupt.publish(msg);
    }
    for (auto&robot: robots_)
    {
      if (req.type.data == robot.type && req.id.data == robot.id)
      {
        if (!robot.plan.empty())
        {
          res.objective = robot.plan[0];
          res.code.data = 1;
          ROS_ERROR_STREAM("Objective " << robot.plan[0]);
        }else
        {
          res.code.data = 0;
        }
      }
    }
  ROS_ERROR("Planning ended.");
  return true;
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

  if (index_pub_scout != 1)
  {
    this->server_task_planner = nh_.advertiseService("/task_planner_scout",&TaskPlanner::taskPlanService,this);
  } else
  {
    this->server_task_planner = nh_.advertiseService("/task_planner_exc_haul",&TaskPlanner::taskPlanService,this);
  }

  pub_interrupt = nh_.advertise<std_msgs::Bool>("planner_interrupt", 1);

  
  this->populate_prior_plan();


}

/////////////////////////////////////////////////////////////////////
/***************************UTILITIES*******************************/
/////////////////////////////////////////////////////////////////////

int TaskPlanner::get_robot_index(int robot_type, int robot_id)
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
}
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