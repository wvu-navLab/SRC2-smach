// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file TaskPlanner.cpp
 */
//-----------------------------------------------------------------------------

#include <task_planning/TaskPlanner.hpp>

namespace mac
{

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
    ROS_INFO("[TASK PLANNER] Populating prior plan...");
    if (!planning_params_.plan.empty())
    {
      for (int i = 0; i < planning_params_.plan[0].size(); ++i)
      {
        geometry_msgs::PointStamped temp;
        for (int j = 0; j < robots_.size(); ++j)
        {
          // ROS_INFO_STREAM("[TASK PLANNER] [" << plan_call_counter << "] Type current:" << robots_[j].type << " ||| plan" << planning_params_.plan[0][i]);
          // ROS_INFO_STREAM("[TASK PLANNER] [" << plan_call_counter << "] ID current:" << robots_[j].id << " ||| plan" << planning_params_.plan[1][i]);
          if (robots_[j].type == planning_params_.plan[0][i] && robots_[j].id == planning_params_.plan[1][i])
          {
            temp.point.x = planning_params_.plan[2][i];
            temp.point.y = planning_params_.plan[3][i];
            temp.point.z = planning_params_.plan[4][i];
            robots_[j].plan.push_back(temp);
            // ROS_INFO_STREAM("[TASK PLANNER] [" << plan_call_counter << "] Plan: (" << temp.point.x << ", " << temp.point.y << ")");
          }
        }
      }
    }
  }

  void TaskPlanner::scout_plan_default(int type, int id)
  {
    for (auto &robot : robots_)
    {
      if (type == robot.type && id == robot.id)
      {
        // ROS_INFO_STREAM("[TASK PLANNER] [" << plan_call_counter << "] Plan " << robot.plan[0]);
        geometry_msgs::PointStamped temp;
        temp = robot.plan[0];
        robot.plan.erase(robot.plan.begin());
        robot.plan.push_back(temp);
        // ROS_INFO_STREAM("[TASK PLANNER] [" << plan_call_counter << "] Plan " << robot.plan[0]);
      }
    }
  }

  void TaskPlanner::exc_haul_plan_default()
  {
    for (auto &robot : robots_)
    {
      robot.plan.clear();
      robot.volatile_indices.clear();
    }

    int nearest_ind, min_distance;
    std::vector<double> pose_min, vol_pose, default_pose, current_pose;
    default_pose.push_back(0);
    default_pose.push_back(0);
    // ROS_ERROR_STREAM("[TASK PLANNER] [" << plan_call_counter << "] Volatile Map Size: " << volatile_map_.vol.size());
    for (int i = 0; i < volatile_map_.vol.size(); ++i)
    {
      // ROS_ERROR_STREAM("[TASK PLANNER] [" << plan_call_counter << "] Started Checking Volatiles");

      if (volatile_map_.vol[i].attempted || !volatile_map_.vol[i].honed)
      {
        continue;
      }

      //EXCAVATOR
      vol_pose = default_pose;
      current_pose = default_pose;
      nearest_ind = -1;
      min_distance = 5000;
      pose_min = default_pose;
      vol_pose[0] = volatile_map_.vol[i].position.point.x;
      vol_pose[1] = volatile_map_.vol[i].position.point.y;

      // ROS_ERROR_STREAM("[TASK PLANNER] [" << plan_call_counter << "] Volatile pose " << vol_pose[0] << "," << vol_pose[1]);
      // std::cout << "robots_.size() = " << robots_.size() << std::endl;
      for (int j = 0; j < robots_.size(); ++j)
      {
        if (robots_[j].type == mac::EXCAVATOR)
        {
          if (nearest_ind != -1)
          {
            current_pose[0] = robots_[j].odom.pose.pose.position.x;
            current_pose[1] = robots_[j].odom.pose.pose.position.y; // robot of nearest in pose
            // ROS_ERROR_STREAM("[TASK PLANNER] [" << plan_call_counter << "] Excavator pose " << current_pose[0] << "," << current_pose[1]);

            // robot of current in pose
            double distance = TaskPlanner::dist(current_pose, vol_pose);
            if (distance < min_distance)
            {
              min_distance = distance;
              nearest_ind = j;
              pose_min = current_pose;
            }
          }
          else if (robots_[j].current_task < 0)
          {
            nearest_ind = j;

            pose_min[0] = robots_[j].odom.pose.pose.position.x;
            pose_min[1] = robots_[j].odom.pose.pose.position.y;
            // ROS_ERROR_STREAM("[TASK PLANNER] [" << plan_call_counter << "] Excavator pose min " << pose_min[0] << "," << pose_min[1]);
          }
        }
      }
      geometry_msgs::PointStamped temp;
      if (vol_pose[0] != 0 || vol_pose[1] != 0)
      {
        temp.point.x = vol_pose[0];
        temp.point.y = vol_pose[1];
        robots_[nearest_ind].volatile_indices.push_back(volatile_map_.vol[i].vol_index);
        robots_[nearest_ind].plan.push_back(temp);
      }

      //HAULER
      vol_pose = default_pose;
      current_pose = default_pose;
      nearest_ind = -1;
      min_distance = 5000;
      pose_min = default_pose;
      vol_pose[0] = volatile_map_.vol[i].position.point.x;
      vol_pose[1] = volatile_map_.vol[i].position.point.y;
      // ROS_ERROR_STREAM("[TASK PLANNER] [" << plan_call_counter << "] Volatile pose " << vol_pose[0] << "," << vol_pose[1]);
      for (int j = 0; j < robots_.size(); ++j)
      {
        if (robots_[j].type == mac::HAULER)
        {
          if (nearest_ind != -1)
          {
            current_pose[0] = robots_[j].odom.pose.pose.position.x;
            current_pose[1] = robots_[j].odom.pose.pose.position.y; // robot of nearest in pose
            // ROS_ERROR_STREAM("[TASK PLANNER] [" << plan_call_counter << "] Hauler pose " << current_pose[0] << "," << current_pose[1]);

            // robot of current in pose
            double distance = TaskPlanner::dist(current_pose, vol_pose);
            if (distance < min_distance)
            {
              min_distance = distance;
              nearest_ind = j;
              pose_min = current_pose;
            }
          }
          else if (robots_[j].current_task < 0)
          {
            nearest_ind = j;
            pose_min[0] = robots_[j].odom.pose.pose.position.x;
            pose_min[1] = robots_[j].odom.pose.pose.position.y;
            // ROS_ERROR_STREAM("[TASK PLANNER] [" << plan_call_counter << "] Hauler pose min " << pose_min[0] << "," << pose_min[1]);
          }
        }
      }
      if (vol_pose[0] != 0 || vol_pose[1] != 0)
      {
        double dx = vol_pose[0] - current_pose[0];
        double dy = vol_pose[1] - current_pose[1];
        double D = hypot(dx, dy);
        temp.point.x = vol_pose[0] - dx / D * 10.0; // TODO: OFFSET FOR HAULER
        temp.point.y = vol_pose[1] - dy / D * 10.0;
        // temp.point.x = vol_pose[0];
        // temp.point.y = vol_pose[1];
        robots_[nearest_ind].volatile_indices.push_back(volatile_map_.vol[i].vol_index);
        robots_[nearest_ind].plan.push_back(temp);
        //robots_[nearest_ind].current_task = (int)mac::ACTION_HAULER_T::_volatile_handler;
      }
    }
  }

  void TaskPlanner::exc_haul_plan_default_adv()
  {
    ros::spinOnce();

    for (auto &robot : robots_)
    {
      robot.plan.clear();
      robot.volatile_indices.clear();
    }

    for (auto &robot : robots_)
    {
      for (auto &vol : volatile_map_.vol)
      {
        if (robot.type == mac::EXCAVATOR && vol.robot_id_assigned == robot.id && !vol.attempted)
        {
          robot.plan.push_back(vol.position);
          robot.volatile_indices.push_back(vol.vol_index);
        }
      }
    }

    volatile_map::VolatileMap temp_map;
    std::vector<int> temp_volatile_indices;
    for (auto &vol : volatile_map_.vol)
    {
      if (vol.attempted || !vol.honed || vol.robot_id_assigned)
      {
        continue;
      }
      temp_map.vol.push_back(vol);
      temp_volatile_indices.push_back(vol.vol_index);
    }

    double x, dx;
    double y, dy;
    double D, min_D, min_D1, min_D2;
    int exc1_ind, exc2_ind, first_exc_ind;
    bool flag_exc1_has_plan, flag_exc2_has_plan;
    int min_ind, min_ind1, min_ind2, vol_ind;

    if (volatile_map_.vol.size() == 1)
    {
      if (volatile_map_.vol[0].honed)
      {
        min_D = 500;
        min_ind = 0;
        for (auto &robot : robots_)
        {
          if (robot.type == mac::EXCAVATOR)
          {
            x = robot.odom.pose.pose.position.x;
            y = robot.odom.pose.pose.position.y;
            dx = volatile_map_.vol[0].position.point.x - x;
            dy = volatile_map_.vol[0].position.point.y - y;
            D = hypot(dx, dy);
            if (D < min_D)
            {
              min_D = D;
              min_ind = robot.id;
            }
          }
        }
        first_exc_ind = get_robot_index(mac::EXCAVATOR, min_ind);
        robots_[first_exc_ind].plan.push_back(volatile_map_.vol[0].position);
        robots_[first_exc_ind].volatile_indices.push_back(volatile_map_.vol[0].vol_index);
      }
    }
    else if (temp_map.vol.size() > 0)
    {
      exc1_ind = get_robot_index(mac::EXCAVATOR, 1);
      exc2_ind = get_robot_index(mac::EXCAVATOR, 2);

      flag_exc1_has_plan = (robots_[exc1_ind].plan.size() > 0);
      flag_exc2_has_plan = (robots_[exc2_ind].plan.size() > 0);

      if (!flag_exc1_has_plan && !flag_exc2_has_plan)
      {
        // Calculate closest vol to excavator 1
        x = robots_[exc1_ind].odom.pose.pose.position.x;
        y = robots_[exc1_ind].odom.pose.pose.position.y;
        min_D = 500;
        min_ind = 0;
        vol_ind = 0;
        for (auto &vol : temp_map.vol)
        {
          dx = vol.position.point.x - x;
          dy = vol.position.point.y - y;
          D = hypot(dx, dy);
          if (D < min_D)
          {
            min_D = D;
            min_ind = vol_ind;
          }
          ++vol_ind;
        }
        min_D1 = min_D;
        min_ind1 = min_ind;

        // Calculate closest vol to excavator 2
        x = robots_[exc2_ind].odom.pose.pose.position.x;
        y = robots_[exc2_ind].odom.pose.pose.position.y;
        min_D = 500;
        min_ind = 0;
        vol_ind = 0;
        for (auto &vol : temp_map.vol)
        {
          dx = vol.position.point.x - x;
          dy = vol.position.point.y - y;
          D = hypot(dx, dy);
          if (D < min_D)
          {
            min_D = D;
            min_ind = vol_ind;
          }
          ++vol_ind;
        }
        min_D2 = min_D;
        min_ind2 = min_ind;
        // If both have the same volatile as the closest
        if (min_ind2 == min_ind1)
        {
          if (min_D1 < min_D2)
          {
            // If its closer to excavator 1
            // Give excavator 1 the closest to both
            robots_[exc1_ind].plan.push_back(temp_map.vol[min_ind1].position);
            robots_[exc1_ind].volatile_indices.push_back(temp_volatile_indices[min_ind1]);

            temp_map.vol.erase(temp_map.vol.begin() + min_ind1);
            temp_volatile_indices.erase(temp_volatile_indices.begin() + min_ind1);

            if (temp_map.vol.size() > 0)
            {
              // Give excavator 2 its second closest
              x = robots_[exc2_ind].odom.pose.pose.position.x;
              y = robots_[exc2_ind].odom.pose.pose.position.y;
              min_D = 500;
              min_ind = 0;
              vol_ind = 0;
              for (auto &vol : temp_map.vol)
              {
                dx = vol.position.point.x - x;
                dy = vol.position.point.y - y;
                D = hypot(dx, dy);
                if (D < min_D)
                {
                  min_D = D;
                  min_ind = vol_ind;
                }
                ++vol_ind;
              }
              robots_[exc2_ind].plan.push_back(temp_map.vol[min_ind].position);
              robots_[exc2_ind].volatile_indices.push_back(temp_volatile_indices[min_ind]);
            }
          }
          else
          {
            // If its closer to excavator 2
            // Give excavator 2 the closest to both
            robots_[exc2_ind].plan.push_back(temp_map.vol[min_ind2].position);
            robots_[exc2_ind].volatile_indices.push_back(temp_volatile_indices[min_ind2]);

            temp_map.vol.erase(temp_map.vol.begin() + min_ind2);
            temp_volatile_indices.erase(temp_volatile_indices.begin() + min_ind2);

            if (temp_map.vol.size() > 0)
            {
              // Give excavator 1 its second closest
              x = robots_[exc1_ind].odom.pose.pose.position.x;
              y = robots_[exc1_ind].odom.pose.pose.position.y;
              min_D = 500;
              min_ind = 0;
              vol_ind = 0;
              for (auto &vol : temp_map.vol)
              {
                dx = vol.position.point.x - x;
                dy = vol.position.point.y - y;
                D = hypot(dx, dy);
                if (D < min_D)
                {
                  min_D = D;
                  min_ind = vol_ind;
                }
                ++vol_ind;
              }
              robots_[exc1_ind].plan.push_back(temp_map.vol[min_ind].position);
              robots_[exc1_ind].volatile_indices.push_back(temp_volatile_indices[min_ind]);
            }
          }
        }
        else
        {
          // Give excavator 1 its closest
          robots_[exc1_ind].plan.push_back(temp_map.vol[min_ind1].position);
          robots_[exc1_ind].volatile_indices.push_back(temp_volatile_indices[min_ind1]);
          // Give excavator 2 its closest
          robots_[exc2_ind].plan.push_back(temp_map.vol[min_ind2].position);
          robots_[exc2_ind].volatile_indices.push_back(temp_volatile_indices[min_ind2]);
        }
      }
      else if (!flag_exc1_has_plan && flag_exc2_has_plan)
      {
        // Calculate closest vol to excavator 1
        x = robots_[exc1_ind].odom.pose.pose.position.x;
        y = robots_[exc1_ind].odom.pose.pose.position.y;
        min_D = 500;
        min_ind = 0;
        vol_ind = 0;
        for (auto &vol : temp_map.vol)
        {
          dx = vol.position.point.x - x;
          dy = vol.position.point.y - y;
          D = hypot(dx, dy);
          if (D < min_D)
          {
            min_D = D;
            min_ind = vol_ind;
          }
          ++vol_ind;
        }
        robots_[exc1_ind].plan.push_back(temp_map.vol[min_ind].position);
        robots_[exc1_ind].volatile_indices.push_back(temp_volatile_indices[min_ind]);
      }
      else if (flag_exc1_has_plan && !flag_exc2_has_plan)
      {
        // Calculate closest vol to excavator 2
        x = robots_[exc2_ind].odom.pose.pose.position.x;
        y = robots_[exc2_ind].odom.pose.pose.position.y;
        min_D = 500;
        min_ind = 0;
        vol_ind = 0;
        for (auto &vol : temp_map.vol)
        {
          dx = vol.position.point.x - x;
          dy = vol.position.point.y - y;
          D = hypot(dx, dy);
          if (D < min_D)
          {
            min_D = D;
            min_ind = vol_ind;
          }
          ++vol_ind;
        }
        robots_[exc2_ind].plan.push_back(temp_map.vol[min_ind].position);
        robots_[exc2_ind].volatile_indices.push_back(temp_volatile_indices[min_ind]);
      }

      // volatile_map::VolatileMap temp_map;
      // std::vector<int> temp_volatile_indices;
      // for (auto &vol : volatile_map_.vol)
      // {
      //   if (vol.attempted || !vol.honed || vol.robot_id_assigned)
      //   {
      //     continue;
      //   }
      //   temp_map.vol.push_back(vol);
      //   temp_volatile_indices.push_back(vol.vol_index);
      // }

      // int exc1_ind = get_robot_index(mac::EXCAVATOR, 1);
      // int exc2_ind = get_robot_index(mac::EXCAVATOR, 2);
      // int counter = 0;

      // while (temp_map.vol.size() > 0)
      // {
      //   for (auto &robot : robots_)
      //   {
      //     if(robot.type == mac::EXCAVATOR && counter == 0 && (robots_[exc2_ind].plan.size() == 0) && (robots_[exc1_ind].plan.size() == 1))
      //     {
      //       counter++;
      //       continue;
      //     }
      //     double x;
      //     double y;
      //     if (robot.type == mac::EXCAVATOR)
      //     {
      //       if (robot.plan.empty())
      //       {
      //         x = robot.odom.pose.pose.position.x;
      //         y = robot.odom.pose.pose.position.y;
      //       }
      //       else
      //       {
      //         x = robot.plan[robot.plan.size() - 1].point.x;
      //         y = robot.plan[robot.plan.size() - 1].point.y;
      //       }
      //       double min_D = 500;
      //       double min_ind = 0;
      //       int vol_ind = 0;
      //       for (auto &vol : temp_map.vol)
      //       {
      //         double dx = vol.position.point.x - x;
      //         double dy = vol.position.point.y - y;
      //         double D = hypot(dx, dy);
      //         if (D < min_D)
      //         {
      //           min_D = D;
      //           min_ind = vol_ind;
      //         }
      //         ++vol_ind;
      //       }
      //       geometry_msgs::PointStamped temp;
      //       temp.point.x = temp_map.vol[min_ind].position.point.x;
      //       temp.point.y = temp_map.vol[min_ind].position.point.y;
      //       robot.plan.push_back(temp);
      //       robot.volatile_indices.push_back(temp_volatile_indices[min_ind]);
      //       temp_map.vol.erase(temp_map.vol.begin() + min_ind);
      //       temp_volatile_indices.erase(temp_volatile_indices.begin() + min_ind);
      //     }
      //     if (temp_map.vol.empty())
      //     {
      //       break;
      //     }
      //   }
      // }
    }

    for (auto &robot : robots_)
    {
      if (robot.type == mac::HAULER)
      {
        int exc_ind = get_robot_index(mac::EXCAVATOR, robot.id);
        robot.volatile_indices = robots_[exc_ind].volatile_indices;
        robot.plan = robots_[exc_ind].plan;

        geometry_msgs::PointStamped temp;
        for (int i = 0; i < robot.plan.size(); ++i)
        {
          double dx = robot.plan[i].point.x - robot.odom.pose.pose.position.x;
          double dy = robot.plan[i].point.y - robot.odom.pose.pose.position.y;
          double D = hypot(dx, dy);
          temp.point.x = robot.plan[i].point.x - dx / D * 10.0; // TODO: OFFSET FOR HAULER
          temp.point.y = robot.plan[i].point.y - dy / D * 10.0;
          // temp.point.x = vol_pose[0];
          // temp.point.y = vol_pose[1];
          //robots_[nearest_ind].volatile_indices.push_back(volatile_map_.vol[i].vol_index);
          robot.plan[i] = temp;
          //robots_[nearest_ind].current_task = (int)mac::ACTION_HAULER_T::_volatile_handler;
        }
      }
    }
  }

  void TaskPlanner::exc_haul_plan_fs(mac::State s)
  {
    for (auto &robot : robots_)
    {
      robot.plan.clear();
      robot.volatile_indices.clear();
    }

    std::vector<mac::Action> joint_act = this->forward_search_.plan(s);

    for (auto &act : joint_act)
    {
      int robot_ind = get_robot_index(act.robot_type, act.id);
      robots_[robot_ind].volatile_indices.clear();
      robots_[robot_ind].volatile_indices.push_back(act.volatile_index);
      geometry_msgs::PointStamped temp;
      temp.point.x = act.objective.first;
      temp.point.y = act.objective.second;
      robots_[robot_ind].plan.push_back(temp);
      robots_[robot_ind].current_task = act.code;
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

  void TaskPlanner::poseCallback(const ros::MessageEvent<nav_msgs::Odometry const> &event)
  {

    const ros::M_string &header = event.getConnectionHeader();
    std::string topic = header.at("topic");
    char robot_type = topic.c_str()[7]; // first character is at 8th index

    int r_type;

    const nav_msgs::OdometryConstPtr &msg = event.getMessage();

    char ind;
    int id;
    switch (robot_type)
    {
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
      ROS_ERROR_STREAM("[TASK PLANNER] [" << plan_call_counter << "] Incorrect Robot Type");
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
      ROS_ERROR_STREAM("Incorrect Robot Type");

  }
  //ROS_WARN("HRMM %s",topic.c_str());
  ROS_WARN("%c %i",robot_type,index);// << std::endl;
  //ROS_DEBUG("%d",msg->data);

}*/
  bool TaskPlanner::dumpRequestService(task_planning::DumpCoordination::Request &req, task_planning::DumpCoordination::Response &res)
  {
    // a hauler wants to dump, and the other hauler has not set the flag
    // accept the request and set the hauler dumping true
    if (req.dump_request && !hauler_dumping)
    {
      hauler_dumping = true;
      res.request_accepted = true;
    }
    // a hauler wants to dump, but the flag that the other hauler is set is already true_pose
    // deny the request
    else if (req.dump_request && hauler_dumping)
    {
      res.request_accepted = false;
    }
    // any hauler can change the status to no longer dumping anytime they want
    else if (!req.dump_request)
    {
      hauler_dumping = false;
      res.request_accepted = true;
    }

    return true;
  }
  
  bool TaskPlanner::taskPlanService(task_planning::PlanInfo::Request &req, task_planning::PlanInfo::Response &res)
  {
    // ROS_ERROR_STREAM("[TASK PLANNER] [" << plan_call_counter << "] Planning started.");
    State s;
    if (req.replan.data)
    {
      // ROS_ERROR_STREAM("[TASK PLANNER] [" << plan_call_counter << "] Planning started.");
      //perform planning type
      switch (planning_params_.type)
      {
      case SCOUT_PLANNER_DEFAULT:
        // ROS_WARN_STREAM("[TASK PLANNER] [" << plan_call_counter << "] SCOUT PLANNER. Planning started.");
        this->scout_plan_default(req.type.data, req.id.data);
        break;
      case EXC_HAUL_PLANNER_DEFAULT:
        // ROS_WARN_STREAM("[TASK PLANNER] [" << plan_call_counter << "] EXC HAUL PLANNER. Planning started.");
        this->exc_haul_plan_default_adv();
        break;
      case EXC_HAUL_FORWARD_SEARCH:
        // ROS_WARN_STREAM("[TASK PLANNER] [" << plan_call_counter << "] EXC HAUL FORWARD SEARCH. Planning started.");
        s.robots = robots_;
        s.volatile_map = volatile_map_;
        s.time = time_;

        this->exc_haul_plan_fs(s);
        break;
      default:
        // ROS_ERROR_STREAM("[TASK PLANNER] [" << plan_call_counter << "] Task Planner type invalid!");
        break;
      }
      std_msgs::Bool msg;
      msg.data = true;
      pub_interrupt.publish(msg);
    }

    switch (planning_params_.type)
    {
    case SCOUT_PLANNER_DEFAULT:
      ROS_WARN_STREAM("[TASK PLANNER] [" << plan_call_counter << "] SCOUT PLANNER. Planning started.");
      break;
    case EXC_HAUL_PLANNER_DEFAULT:
      ROS_WARN_STREAM("[TASK PLANNER] [" << plan_call_counter << "] EXC_HAUL PLANNER. Planning started.");
      break;
    default:
      break;
    }

    ROS_INFO_STREAM("[TASK PLANNER] [" << plan_call_counter << "] REQUEST robot.type " << (int)req.type.data << ", robot.id " << (int)req.id.data);

    res.code.data = 255;

    for (auto &robot : robots_)
    {
      // ROS_INFO_STREAM("[TASK PLANNER] [" << plan_call_counter << "] Checking robot.type " << robot.type << ", robot.id" << robot.id);

      if (!robot.plan.empty())
      {
        // ROS_WARN_STREAM("[TASK PLANNER] [" << plan_call_counter << "] Checking plan: (" << robot.plan[0].point.x << ", " << robot.plan[0].point.y << ", " << robot.plan[0].point.z << ")");
      }

      if (req.type.data == robot.type && req.id.data == robot.id)
      {
        if (!robot.plan.empty())
        {
          if (robot.type == mac::SCOUT)
          {
            res.code.data = robot.plan[0].point.z;
            res.volatile_index.data = 0;
          }
          else
          {
            res.code.data = 3;
            res.volatile_index.data = robot.volatile_indices[0];
            // ROS_WARN_STREAM("[TASK PLANNER] [" << plan_call_counter << "] Volatile index sent " << robot.volatile_indices[0]);
          }
          //--------------------------------------------
          // if (robot.type == mac::HAULER && robot.current_task == (int)ACTION_HAULER_T::_volatile_handler)
          // {
          //   double dx = robot.plan[0].point.x  - robot.odom.pose.pose.position.x;
          //   double dy = robot.plan[0].point.y  - robot.odom.pose.pose.position.y;
          //   double D = hypot(dx,dy);
          //   geometry_msgs::PointStamped temp;
          //   temp.point.x = robot.plan[0].point.x  - dx/D * 10.0; // TODO: OFFSET FOR HAULER
          //   temp.point.y = robot.plan[0].point.y  - dy/D * 10.0;
          //   robot.plan[0].point.x = temp.point.x;
          //   robot.plan[0].point.y = temp.point.y;
          // }
          //--------------------------------------------
          ROS_INFO_STREAM("[TASK PLANNER] [" << plan_call_counter << "] Objective sent. x: " << robot.plan[0].point.x << ", y: " << robot.plan[0].point.y);
          res.objective = robot.plan[0];
          res.objective.point.z = 0;
        }
        else
        {
          ROS_ERROR_STREAM("[TASK PLANNER] [" << plan_call_counter << "] No objective.");
        }
      }
    }
    ROS_INFO_STREAM("[TASK PLANNER] [" << plan_call_counter << "] Planning ended. Code sent " << (int)res.code.data);

    // ROS_ERROR_STREAM("[TASK PLANNER] [" << plan_call_counter << "] Planning ended.");

    plan_call_counter++;

    return true;
  }

  /////////////////////////////////////////////////////////////////////
  /***************************CONSTRUCTORS****************************/
  /////////////////////////////////////////////////////////////////////

  TaskPlanner::TaskPlanner(const CostFunction &cost_function,
                           const std::vector<Robot> &robots,
                           const PlanningParams &planning_params) : cost_function_(cost_function), robots_(robots), planning_params_(planning_params)
  {
    //setup robot subscribers
    int index_sub_scout = 1;
    int index_sub_excavator = 1;
    int index_sub_hauler = 1;

    std::string topic;
    std::string localization_topic;
    if (planning_params.demo)
    {
      localization_topic = "/localization/odometry/sensor_fusion";
    }
    else
    {
      localization_topic = "/localization/odometry/sensor_fusion";
    }

    for (int i = 0; i < robots.size(); i++)
    {
      switch (robots[i].type)
      {
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
        ROS_ERROR_STREAM("[TASK PLANNER] [" << plan_call_counter << "] Robot type invalid!");
        break;
      }
    }

    //setup robot publishers
    int index_pub_scout = 1;
    int index_pub_excavator = 1;
    int index_pub_hauler = 1;
    std::string task_topic = "/task";
    for (int i = 0; i < robots.size(); i++)
    {
      switch (robots[i].type)
      {
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
        ROS_ERROR_STREAM("[TASK PLANNER] [" << plan_call_counter << "] Robot type invalid!");
        break;
      }
    }

    sub_clock_ = nh_.subscribe("/clock", 10, &TaskPlanner::timeCallback, this);
    sub_volatiles_ = nh_.subscribe("/volatile_map", 10, &TaskPlanner::volatileMapCallback, this);

    if (index_pub_scout != 1)
    {
      this->server_task_planner = nh_.advertiseService("/task_planner/scout", &TaskPlanner::taskPlanService, this);
    }
    else
    {
      this->server_task_planner = nh_.advertiseService("/task_planner/exc_haul", &TaskPlanner::taskPlanService, this);
    }

    pub_interrupt = nh_.advertise<std_msgs::Bool>("/task_planner/interrupt", 1);

    this->populate_prior_plan();

    forward_search_.set_cost_function(cost_function);
    forward_search_.set_planning_params(planning_params);

    server_dump_request = nh_.advertiseService("/task_planner/dump_coordination", &TaskPlanner::dumpRequestService, this);
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

  double TaskPlanner::dist(const std::vector<double> p1, const std::vector<double> p2)
  {
    if (p1.size() != p2.size())
    {
      // std::cout << "Error! p1.size() != p2.size() for computing distance!\n";
      exit(1); //TODO: remove exit
    }
    double val = 0;
    for (int i = 0; i < p1.size(); i++)
    {
      double diff = p1[i] - p2[i];
      val = val + diff * diff;
    }
    val = std::sqrt(val);
    return val;
  }

}
