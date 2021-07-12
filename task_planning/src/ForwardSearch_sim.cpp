#include <task_planning/ForwardSearch.hpp>

namespace mac
{

  std::vector<double> ForwardSearch::actions_to_time(const State &s,
                                                     const std::vector<Action> &joint_action)
  {
    //std::cout << "ACTIONS TO TIME" << std::endl;

    std::vector<double> time;
    for (auto &a : joint_action)
    {
      time.push_back(action_to_time(s, a));
    }
    return time;
  }

  double ForwardSearch::action_to_time(const State &s,
                                       const Action &a)
  {
    //std::cout << "ACTION TO TIME" << std::endl;

    //get robot
    int robot_ind = get_robot_index(s, a.robot_type, a.id);
    Robot robot = s.robots[robot_ind];
    double traverse_time, handling_time, homing_time, dumping_time;

    if (robot.type == SCOUT)
    {
      switch (a.code)
      {
      case (int)ACTION_SCOUT_T::_planning:
        return planning_params_.wait_time + 5 * unif_(rng_); // add RNG
        break;
      case (int)ACTION_SCOUT_T::_volatile_handler:
        traverse_time = get_traverse_time(robot);
        handling_time = get_handling_time(robot);
        return traverse_time + handling_time + 5 * unif_(rng_);
        break;
      case (int)ACTION_SCOUT_T::_lost:
        traverse_time = get_traverse_time(robot);
        homing_time = get_homing_time(robot);
        return traverse_time + homing_time + 5 * unif_(rng_);
        break;
      default:
        ROS_ERROR("Invalid robot type: action_to_time");
        break;
      }
    }

    if (robot.type == EXCAVATOR)
    {
      switch (a.code)
      {
      case (int)ACTION_EXCAVATOR_T::_planning:
        //std::cout << "action_to_time: exc plan" << std::endl;

        return planning_params_.wait_time + 5 * unif_(rng_);
        break;
      case (int)ACTION_EXCAVATOR_T::_volatile_handler:
        //std::cout << "action_to_time: exc vol: trav" << std::endl;

        traverse_time = get_traverse_time(robot) + 5 * unif_(rng_);
        //  std::cout << "action_to_time: handle" << std::endl;

        handling_time = get_handling_time(robot) + 5 * unif_(rng_);
        return traverse_time + handling_time;
        break;
      case (int)ACTION_EXCAVATOR_T::_lost:
        // std::cout << "action_to_time: exc lost: trav" << std::endl;

        traverse_time = get_traverse_time(robot) + 5 * unif_(rng_);
        //   std::cout << "action_to_time: home" << std::endl;

        homing_time = get_homing_time(robot);
        return traverse_time + homing_time + 5 * unif_(rng_);
        break;
      default:
        ROS_ERROR("Invalid robot type: action_to_time");
        break;
      }
    }

    if (robot.type == HAULER)
    {
      switch (a.code)
      {
      case (int)ACTION_HAULER_T::_planning:
        return planning_params_.wait_time + 5 * unif_(rng_);
        break;
      case (int)ACTION_HAULER_T::_volatile_handler:
        traverse_time = get_traverse_time(robot);
        handling_time = get_handling_time(robot);
        return traverse_time + handling_time + 5 * unif_(rng_);
        break;
      case (int)ACTION_HAULER_T::_lost:
        traverse_time = get_traverse_time(robot);
        homing_time = get_homing_time(robot);
        return traverse_time + homing_time + 5 * unif_(rng_);
        break;
      case (int)ACTION_HAULER_T::_hauler_dumping:
        traverse_time = get_traverse_time(robot);
        dumping_time = get_dumping_time(robot);
        return traverse_time + dumping_time + 5 * unif_(rng_);
        break;
      default:
        ROS_ERROR("Invalid robot type: action_to_time");
        break;
      }
    }

    return -1;
  }

  double ForwardSearch::get_traverse_time(Robot &robot)
  {
    //std::cout << "GET TRAVERSE TIME" << std::endl;
    double v;
    switch (robot.type)
    {
    case mac::SCOUT:
      v = planning_params_.max_v_scout;
      break;
    case mac::EXCAVATOR:
      v = planning_params_.max_v_excavator;
      break;
    case mac::HAULER:
      v = planning_params_.max_v_hauler;
      break;
    default:
      ROS_ERROR("Invalid robot type: get_traverse_time");
      break;
    }

    if (robot.toggle_sleep)
      v *= 0.75;

    std::vector<double> p1, p2;
    //ROS_INFO_STREAM(robot.odom.pose.pose);
    p1.push_back(robot.odom.pose.pose.position.x);
    p1.push_back(robot.odom.pose.pose.position.y);
    //ROS_INFO_STREAM(robot.plan[0]);
    p2.push_back(robot.plan[0].point.x);
    p2.push_back(robot.plan[0].point.y);

    return this->dist(p1, p2) / v;
  }

  double ForwardSearch::get_handling_time(Robot &robot)
  {
    //std::cout << "GET HANDLING TIME" << std::endl;

    switch (robot.type)
    {
    case mac::SCOUT:
      return planning_params_.vol_handling_time_scout;
    case mac::EXCAVATOR:
      return planning_params_.vol_handling_time_excavator;
    case mac::HAULER:
      return planning_params_.vol_handling_time_hauler;
    default:
      ROS_ERROR("Invalid robot type: get_handling_time");
      break;
    }
    return -1;
  }

  double ForwardSearch::get_homing_time(Robot &robot)
  {
    //std::cout << "GET HOMING TIME" << std::endl;

    switch (robot.type)
    {
    case mac::SCOUT:
      return planning_params_.homing_time_scout;
    case mac::EXCAVATOR:
      return planning_params_.homing_time_excavator;
    case mac::HAULER:
      return planning_params_.homing_time_hauler;
    default:
      ROS_ERROR("Invalid robot type: get_traverse_time");
      break;
    }
    return -1;
  }

  double ForwardSearch::get_dumping_time(Robot &robot)
  {
    //std::cout << "GET DUMPING TIME" << std::endl;

    return planning_params_.dumping_time;
  }

  void ForwardSearch::simulate_time_step(Robot &robot,
                                         double time)
  {
    //std::cout << "SIMULATE TIME STEP" << std::endl;

    // simulate time change
    robot.time_remaining = robot.time_remaining - time;
    if (robot.time_remaining < 0)
      robot.time_remaining = 0;
    // std::cout << "simulate_time_step: time remaining" << std::endl;
    // simulate power
    // this->time_to_power(robot, time);
    // // simulate position and uncertainty
    this->time_to_motion(robot, time);
    // std::cout << "simulate_time_step: after time to motion" << std::endl;
  }

  double ForwardSearch::time_to_motion(Robot &robot,
                                       double time)
  {
    //std::cout << "TIME TO MOTION" << std::endl;

    // if position differs from target increment in direction, else do nothing
    double v;
    switch (robot.type)
    {
    case mac::SCOUT:
      v = planning_params_.max_v_scout;
      break;
    case mac::EXCAVATOR:
      v = planning_params_.max_v_excavator;
      break;
    case mac::HAULER:
      v = planning_params_.max_v_hauler;
      break;
    default:
      ROS_ERROR("Invalid robot type: get_traverse_time");
      v = -1;
      break;
    }

    if (robot.toggle_sleep)
      v *= 0.75;

    // if (robot.plan.empty() )
    // {
    //  std::vector<geometry_msgs::PointStamped> temp_plan;
    //  temp_plan.point.x = robot.odom.pose.pose.position.x;
    //  temp_plan.point.y = robot.odom.pose.pose.position.y;
    //  robot.plan.push_back(temp_plan);
    // }

    // std::cout << "printing time to motion 1" << std::endl;
    // std::cout << "time_to_motion: robot (" << robot.odom.pose.pose.position.x << "," << robot.odom.pose.pose.position.y << ")" << std::endl;
    // std::cout << "time_to_motion: volatile (" << robot.plan[0].point.x << "," << robot.plan[0].point.y << ")" << std::endl;

    std::vector<double> p1, p2;
    p1.push_back(robot.odom.pose.pose.position.x);
    p1.push_back(robot.odom.pose.pose.position.y);
    p2.push_back(robot.plan[0].point.x);
    p2.push_back(robot.plan[0].point.y);

    // std::cout << "printing time to motion 2" << std::endl;
    // std::cout << "time_to_motion: p1 (" << p1[0] << "," << p1[1] << ")" << std::endl;
    // std::cout << "time_to_motion: p2 (" << p2[0] << "," << p2[1] << ")" << std::endl;

    if (p1[0] != p2[0] || p1[1] != p2[1])
    {
      double d = v * time / this->dist(p1, p2);
      if (d <= 1.0)
      {
        robot.odom.pose.pose.position.x += (p2[0] - p1[0]) * d;
        robot.odom.pose.pose.position.y += (p2[1] - p1[1]) * d;
      }
      else
      {
        robot.odom.pose.pose.position.x = p2[0];
        robot.odom.pose.pose.position.y = p2[1];
      }
    }

    return 0;

    // if (robot.type == SCOUT)
    // {
    //   switch (robot->current_task)
    //   {
    //   case ACTION_SCOUT_T::_planning:
    //     return; // no change in position or uncertainty
    //     break;
    //   case ACTION_SCOUT_T::_volatile_handler:
    //     // no clear way account change in position
    //     return;
    //     break;
    //   case ACTION_SCOUT_T::_lost:
    //     return;
    //     break;
    //   default:
    //     ROS_ERROR("Invalid robot code: time_to_motion");
    //     break;
    //   }
    // }

    // if (robot.type == EXCAVATOR)
    // {
    //   switch (robot->current_task)
    //   {
    //   case ACTION_HAULER_T::_planning:
    //     return; // no change in position or uncertainty
    //     break;
    //   case ACTION_HAULER_T::_volatile_handler:
    //     // no clear way account change in position
    //     return;
    //     break;
    //   case ACTION_HAULER_T::_lost:
    //     return;
    //     break;
    //   default:
    //     ROS_ERROR("Invalid robot code: time_to_motion");
    //     break;
    //   }
    // }
    // if (robot.type == HAULER)
    // {
    //   switch (robot->current_task)
    //   {
    //   case ACTION_HAULER_T::_planning:
    //     return; // no change in position or uncertainty
    //     break;
    //   case ACTION_HAULER_T::_volatile_handler:
    //     // no clear way account change in position
    //     return;
    //     break;
    //   case ACTION_HAULER_T::_lost:
    //     return;
    //     break;
    //   case ACTION_HAULER_T::_hauler_dumping:
    //     return;
    //     break;
    //   default:
    //     ROS_ERROR("Invalid robot code: time_to_motion");
    //     break;
    //   }
    // }
  }

}