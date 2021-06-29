#include <task_planning/ForwardSearch.hpp>

namespace mac
{

  /////////////////////////////////////////////////////////////////////
  /***************************CONSTRUCTORS****************************/
  /////////////////////////////////////////////////////////////////////

  ForwardSearch::ForwardSearch() {}

  ForwardSearch::ForwardSearch(const CostFunction &cost_function,
                               const PlanningParams &planning_params)
      : cost_function_(cost_function),
        planning_params_(planning_params) {}

  /////////////////////////////////////////////////////////////////////
  /***************************GETTERS*********************************/
  /////////////////////////////////////////////////////////////////////

  // void ForwardSearch::get_state(const std::vector<Robot> &robots,
  //                               const volatile_map::VolatileMap &volatile_map,
  //                               const ros::Time &time)
  // {
  //   state_.robots = robots;
  //   state_.volatile_map_ = volatile_map;
  //   state_.time_ = time;
  // }

  /////////////////////////////////////////////////////////////////////
  /**************************CORE FUNCTIONALITY***********************/
  /////////////////////////////////////////////////////////////////////
  std::vector<Action> ForwardSearch::plan(const State &s)
  {
    //initialize tree
    tree_.clear();
    Vertex root;
    root.state = s;
    root.cost = 0;
    root.total_cost = 0;
    root.parent_layer_index = -1;
    root.depth = 0;
    root.layer_index = 0;
    root.children.clear();
    std::vector<Vertex> layer_root;
    layer_root.push_back(root);
    tree_.push_back(layer_root);

    //construct tree
    int depth = 0;
    while (depth < planning_params_.max_depth - 1)
    {
      std::vector<Vertex> layer;
      for (auto &leaf : tree_[depth])
      {
        std::vector<std::vector<Action>> joint_actions = get_actions_all_robots(leaf.state);
        for (auto &joint_action : joint_actions)
        {
          Vertex v_new;
          v_new.state = propagate(leaf.state, joint_action);
          v_new.joint_action = joint_action;
          v_new.cost = cost_function_.compute_cost(leaf.state, joint_action, v_new.state);
          v_new.total_cost = leaf.total_cost + v_new.cost;
          v_new.parent_layer_index = leaf.layer_index;
          v_new.depth = tree_.size();
          v_new.layer_index = layer.size();
          v_new.children.clear();
          layer.push_back(v_new);
        }
      }

      if (!layer.empty())
      {
        tree_.push_back(layer);
      }
      else
      {
        std::cout << "No possible actions at current depth. Terminating constructing tree." << std::endl;
        break;
      }

      layer.clear();
      ++depth;
    }

    return get_policy();
  }

  // bool ForwardSearch::reinit()
  // {
  // }

  State ForwardSearch::propagate(const State &s,
                                 const std::vector<Action> &joint_action)
  {
    State s_copy = s;

    //foreach action
    for (auto &a : joint_action)
    {
      int robot_ind = get_robot_index(s_copy, a.robot_type, a.id); //NOTE what if there are multiple
      s_copy.robots[robot_ind].time_remaining = 0;
      s_copy.robots[robot_ind].current_task = a.code;
      s_copy.robots[robot_ind].plan.clear();
      geometry_msgs::PointStamped temp_msg;
      temp_msg.point.x = a.objective.first;
      temp_msg.point.y = a.objective.second;
      s_copy.robots[robot_ind].plan.push_back(temp_msg);
      s_copy.robots[robot_ind].toggle_sleep = a.toggle_sleep;
      s_copy.robots[robot_ind].volatile_index = a.volatile_index;
    }

    // Find time remaining for each robot's task
    std::vector<double> time_remaining = actions_to_time(s_copy, joint_action);

    // Find min time remaining
    int min_time_idx = std::min_element(time_remaining.begin(), time_remaining.end()) - time_remaining.begin();
    double min_time = time_remaining[min_time_idx];
    // Update volatile completion if necessary
    int temp_vol_ind = s_copy.robots[min_time_idx].volatile_index;

    // Need to come back and change this to hauler dumping... For now we are only handling "Excavation"
    if (s_copy.robots[min_time_idx].current_task == (int)ACTION_EXCAVATOR_T::_volatile_handler)
    {
      s_copy.volatile_map.vol[temp_vol_ind].collected = true;
    }
    // propagate completion for each robot
    for (auto &robot : s_copy.robots)
    {
      simulate_time_step(robot, min_time);
    }

    return s_copy;
  }

  std::vector<std::vector<Action>> ForwardSearch::get_actions_all_robots(const State &s)
  {
    //is there volatiles, and is the robots at home, then return no actionss at least for haulers and excavators

    //get all possible actions for individual robots
    std::vector<std::vector<Action>> all_robots_actions;
    std::vector<std::vector<int>> all_robots_actions_indices;
    for (int i = 0; i < s.robots.size(); i++)
    {
      //all possible actions for robot i
      std::vector<Action> robot_actions = get_actions_robot(i, s);
      all_robots_actions.push_back(robot_actions);

      //indices of actions need for cartesian product
      std::vector<int> robot_actions_indices;
      std::iota(robot_actions_indices.begin(), robot_actions_indices.end(), 0);
      all_robots_actions_indices.push_back(robot_actions_indices);
    }

    //get all joint actions from indices
    std::vector<std::vector<int>> all_joints_actions_indices;
    all_joints_actions_indices = cartesian_product(all_robots_actions_indices);

    //populate joint actions from cartesian product
    std::vector<std::vector<Action>> all_joint_actions;
    for (int i = 0; i < all_joints_actions_indices.size(); i++)
    {
      std::vector<Action> joint_action;
      for (int j = 0; j < all_joints_actions_indices[i].size(); j++)
      {
        joint_action.push_back(all_robots_actions[j][all_joints_actions_indices[i][j]]);
      }
    }
    return all_joint_actions;
  }

  std::vector<Action> ForwardSearch::get_actions_robot(int robot_index, const State &s)
  {
    std::vector<Action> actions;

    Robot robot = s.robots[robot_index];

    //get actions for SCOUT
    if (robot.type == SCOUT)
    {
      actions = get_actions_scout(robot_index, s);
    }

    //get actions for EXCAVATOR
    if (robot.type == EXCAVATOR)
    {
      actions = get_actions_excavator(robot_index, s);
    }

    //get actions for HAULER
    if (robot.type == HAULER)
    {
      actions = get_actions_hauler(robot_index, s);
    }

    //add actions for toggle_sleep = true
    std::vector<Action> copy_actions = actions;
    for (const auto &action : copy_actions)
    {
      Action copy_action = action;
      copy_action.toggle_sleep = true;
      actions.push_back(copy_action);
    }

    return actions;
  }

  std::vector<Action> ForwardSearch::get_actions_scout(int robot_index, const State &s)
  {
    std::vector<Action> actions;
    Robot robot = s.robots[robot_index];

    //if task is in progress, do not return any actions
    if (robot.time_remaining < 1e-6)
    {
      actions.clear();
      return actions;
    }

    //TODO: the scout is not currently considered in this planner
    actions.clear();
    return actions;
  }

  std::vector<Action> ForwardSearch::get_actions_excavator(int robot_index, const State &s)
  {
    std::vector<Action> actions;
    Robot robot = s.robots[robot_index];

    //if task is in progress, do not return any actions
    if (robot.time_remaining < 1e-6)
    {
      actions.clear();
      return actions;
    }

    //add indices of volatiles being pursued by other excavators to blacklist
    std::vector<int> vol_blacklist;
    for (const auto &r : s.robots)
    {
      if (r.type != EXCAVATOR)
      {
        continue;
      }
      if (r.volatile_index != -1)
      {
        vol_blacklist.push_back(r.volatile_index);
      }
    }

    //actions VOLATILES
    int volatile_index = 0;
    for (const auto &vol : s.volatile_map.vol)
    {
      //skip blacklisted volatiles
      bool is_vol_blacklisted = false;
      for (const auto vol_bl : vol_blacklist)
      {
        if (volatile_index == vol_bl) //
        {
          is_vol_blacklisted = true;
        }
      }
      if (is_vol_blacklisted)
      {
        continue;
      }

      //add actions for volatiles not in blacklist
      if (!vol.collected)
      {
        Action a;
        a.objective = std::make_pair(vol.position.point.x,
                                     vol.position.point.y);
        a.robot_type = robot.type;
        a.id = robot_index;
        a.code = (int)ACTION_EXCAVATOR_T::_volatile_handler;
        a.volatile_index = volatile_index;
        a.toggle_sleep = false;
        actions.push_back(a);
      }
      volatile_index++;
    }

    //action LOST
    {
      Action a;
      a.objective = std::make_pair(0, 0);
      a.robot_type = robot.type;
      a.id = robot_index;
      a.code = (int)ACTION_EXCAVATOR_T::_lost;
      a.volatile_index = -1;
      a.toggle_sleep = false;
      actions.push_back(a);
    }

    //action PLANNING (i.e., wait action)
    {
      Action a;
      a.objective = std::make_pair(robot.odom.pose.pose.position.x,
                                   robot.odom.pose.pose.position.y);
      a.robot_type = robot.type;
      a.id = robot_index;
      a.code = (int)ACTION_EXCAVATOR_T::_planning;
      a.volatile_index = -1;
      a.toggle_sleep = false;
      actions.push_back(a);
    }
  }

  std::vector<Action> ForwardSearch::get_actions_hauler(int robot_index, const State &s)
  {
    std::vector<Action> actions;
    Robot robot = s.robots[robot_index];

    //if task is in progress, do not return any actions
    if (robot.time_remaining < 1e-6)
    {
      actions.clear();
      return actions;
    }

    //only consider volatiles that are being pursued by excavators
    std::vector<int> vol_indices;
    for (const auto &r : s.robots)
    {
      if (r.type == EXCAVATOR)
      {
        if (r.volatile_index == -1)
        {
          continue;
        }
        else
        {
          vol_indices.push_back(r.volatile_index);
        }
      }
    }

    //actions VOLATILES
    for (const auto &vol_index : vol_indices)
    {
      Action a;
      a.objective = std::make_pair(s.volatile_map.vol[vol_index].position.point.x,
                                   s.volatile_map.vol[vol_index].position.point.y);
      a.robot_type = robot.type;
      a.id = robot_index;
      a.code = (int)ACTION_HAULER_T::_volatile_handler;
      a.volatile_index = vol_index;
      a.toggle_sleep = false;
      actions.push_back(a);
    }

    //action LOST
    {
      Action a;
      a.objective = std::make_pair(0, 0);
      a.robot_type = robot.type;
      a.id = robot_index;
      a.code = (int)ACTION_HAULER_T::_lost;
      a.volatile_index = -1;
      a.toggle_sleep = false;
      actions.push_back(a);
    }
    //action PLANNING (i.e., wait action)
    {
      Action a;
      a.objective = std::make_pair(robot.odom.pose.pose.position.x,
                                   robot.odom.pose.pose.position.y);
      a.robot_type = robot.type;
      a.id = robot_index;
      a.code = (int)ACTION_HAULER_T::_planning;
      a.volatile_index = -1;
      a.toggle_sleep = false;
      actions.push_back(a);
    }
    //action DUMP
    {
      Action a;
      a.objective = std::make_pair(0, 0);
      a.robot_type = robot.type;
      a.id = robot_index;
      a.code = (int)ACTION_HAULER_T::_hauler_dumping;
      a.volatile_index = -1;
      a.toggle_sleep = false;
      actions.push_back(a);
    }
  }

  std::vector<Action> ForwardSearch::get_policy()
  {
    //select leaf with minimum cost (only works if problem is deterministic)
    int min_layer_index = -1;
    int min_depth = -1;
    double min_total_cost = 1e9;
    for (auto &v : tree_[tree_.size() - 1])
    {
      if (min_total_cost < v.total_cost)
      {
        min_layer_index = v.layer_index;
        min_depth = v.depth;
        min_total_cost = v.total_cost;
      }
    }

    if (min_layer_index == -1 || min_depth == -1)
    {
      std::cout << "get_policy: Error! No leaf selected in get_policy!" << std::endl;
    }

    //find optimal joint action given selected leaf
    std::vector<Action> optimal_joint_action;
    bool is_action_valid = false;
    Vertex leaf = tree_[min_depth][min_layer_index];
    while (leaf.depth != 0)
    {
      optimal_joint_action = leaf.joint_action;
      is_action_valid = true;
      leaf = tree_[leaf.depth - 1][leaf.parent_layer_index];
    }

    if (!is_action_valid)
    {
      std::cout << "get_policy: Error! No valid actions selected! Is the tree a depth of 0?." << std::endl;
    }

    return optimal_joint_action;
  }

  /////////////////////////////////////////////////////////////////////
  /***************************UTILITIES*******************************/
  /////////////////////////////////////////////////////////////////////
  // populates time remaing for tasks
  std::vector<double> ForwardSearch::actions_to_time(const State &s,
                                                     const std::vector<Action> &joint_action)
  {
    std::vector<double> time;
    for (auto &a : joint_action)
    {
      time.push_back(action_to_time(s, a));
    }
  }

  double ForwardSearch::action_to_time(const State &s,
                                       const Action &a)
  {
    //get robot
    Robot robot = s.robots[a.id];

    double traverse_time, handling_time, homing_time, dumping_time;

    if (robot.type == SCOUT)
    {
      switch (a.code)
      {
      case (int)ACTION_SCOUT_T::_planning:
        return planning_params_.wait_time;
        break;
      case (int)ACTION_SCOUT_T::_volatile_handler:
        traverse_time = get_traverse_time(robot);
        handling_time = get_handling_time(robot);
        return traverse_time + handling_time;
        break;
      case (int)ACTION_SCOUT_T::_lost:
        traverse_time = get_traverse_time(robot);
        homing_time = get_homing_time(robot);
        return traverse_time + homing_time;
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
        return planning_params_.wait_time;
        break;
      case (int)ACTION_EXCAVATOR_T::_volatile_handler:
        traverse_time = get_traverse_time(robot);
        handling_time = get_handling_time(robot);
        return traverse_time + handling_time;
        break;
      case (int)ACTION_EXCAVATOR_T::_lost:
        traverse_time = get_traverse_time(robot);
        homing_time = get_homing_time(robot);
        return traverse_time + homing_time;
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
        return planning_params_.wait_time;
        break;
      case (int)ACTION_HAULER_T::_volatile_handler:
        traverse_time = get_traverse_time(robot);
        handling_time = get_handling_time(robot);
        return traverse_time + handling_time;
        break;
      case (int)ACTION_HAULER_T::_lost:
        traverse_time = get_traverse_time(robot);
        homing_time = get_homing_time(robot);
        return traverse_time + homing_time;
        break;
      case (int)ACTION_HAULER_T::_hauler_dumping:
        traverse_time = get_traverse_time(robot);
        dumping_time = get_dumping_time(robot);
        return traverse_time + dumping_time;
        break;
      default:
        ROS_ERROR("Invalid robot type: action_to_time");
        break;
      }
    }
  }

  double ForwardSearch::get_traverse_time(Robot &robot)
  {
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
    p1.push_back(robot.odom.pose.pose.position.x);
    p1.push_back(robot.odom.pose.pose.position.y);
    p2.push_back(robot.plan[0].point.x);
    p2.push_back(robot.plan[0].point.y);

    return this->dist(p1, p2) / v;
  }

  double ForwardSearch::get_handling_time(Robot &robot)
  {
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
  }
  double ForwardSearch::get_homing_time(Robot &robot)
  {
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
  }

  double ForwardSearch::get_dumping_time(Robot &robot)
  {
    return planning_params_.dumping_time;
  }

  //-----------------------------------------------------------------------------
  // given timestep update state
  void ForwardSearch::simulate_time_step(Robot &robot,
                                         double time)
  {
    // simulate time change
    robot.time_remaining -= time;
    // simulate power
    // this->time_to_power(robot, time);
    // // simulate position and uncertainty
    this->time_to_motion(robot, time);
  }

  // double ForwardSearch::time_to_power(Robot &robot,
  //                                     double time)
  // {
  //   if (robot.type == SCOUT)
  //   {
  //     switch (robot->current_task)
  //     {
  //     case ACTION_SCOUT_T::_planning:
  //       for (int i = 0; i < planning_params_.power_rates.size(); ++i)
  //         robot.power -= planning_params_.planning_scout_power_weights[i] * planning_params_.power_rates[i] * time;
  //       return robot.power;
  //     case ACTION_SCOUT_T::_volatile_handler:
  //       for (int i = 0; i < planning_params_.power_rates.size(); ++i)
  //         robot.power -= planning_params_.vol_handle_scout_power_weights[i] * planning_params_.power_rates[i] * time;
  //       return;
  //     case ACTION_SCOUT_T::_lost:
  //       for (int i = 0; i < planning_params_.power_rates.size(); ++i)
  //         robot.power -= planning_params_.lost_scout_power_weights[i] * planning_params_.power_rates[i] * time;
  //       return;
  //     default:
  //       ROS_ERROR("Invalid robot code: time_to_power");
  //       break;
  //     }
  //   }

  //   if (robot.type == EXCAVATOR)
  //   {
  //     switch (robot->current_task)
  //     {
  //     case ACTION_EXCAVATOR_T::_planning:
  //       for (int i = 0; i < planning_params_.power_rates.size(); ++i)
  //         robot.power -= planning_params_.planning_excavator_power_weights[i] * planning_params_.power_rates[i] * time;
  //       return;
  //     case ACTION_EXCAVATOR_T::_volatile_handler:
  //       for (int i = 0; i < planning_params_.power_rates.size(); ++i)
  //         robot.power -= planning_params_.vol_handle_excavator_power_weights[i] * planning_params_.power_rates[i] * time;
  //       return;
  //     case ACTION_EXCAVATOR_T::_lost:
  //       for (int i = 0; i < planning_params_.power_rates.size(); ++i)
  //         robot.power -= planning_params_.lost_excavator_power_weights[i] * planning_params_.power_rates[i] * time;
  //       return;
  //     default:
  //       ROS_ERROR("Invalid robot code: time_to_power");
  //       break;
  //     }
  //   }

  //   if (robot.type == HAULER)
  //   {
  //     switch (robot->current_task)
  //     {
  //     case ACTION_HAULER_T::_planning:
  //       for (int i = 0; i < planning_params_.power_rates.size(); ++i)
  //         robot.power -= planning_params_.planning_hauler_power_weights[i] * planning_params_.power_rates[i] * time;
  //       return;
  //     case ACTION_HAULER_T::_volatile_handler:
  //       for (int i = 0; i < planning_params_.power_rates.size(); ++i)
  //         robot.power -= planning_params_.vol_handle_hauler_power_weights[i] * planning_params_.power_rates[i] * time;
  //       return;
  //     case ACTION_HAULER_T::_lost:
  //       for (int i = 0; i < planning_params_.power_rates.size(); ++i)
  //         robot.power -= planning_params_.lost_hauler_power_weights[i] * planning_params_.power_rates[i] * time;
  //       return;
  //     default:
  //       ROS_ERROR("Invalid robot code: time_to_power");
  //       break;
  //     }
  //   }
  // }

  double ForwardSearch::time_to_motion(Robot &robot,
                                       double time)
  {
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

    std::vector<double> p1, p2;
    p1.push_back(robot.odom.pose.pose.position.x);
    p1.push_back(robot.odom.pose.pose.position.y);
    p2.push_back(robot.plan[0].point.x);
    p2.push_back(robot.plan[0].point.y);

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

  //------------------------------------------------------------------

  int ForwardSearch::get_robot_index(const State &s, int robot_type, int robot_id)
  {
    int index = -1;
    for (int i = 0; i < s.robots.size(); ++i)
    {
      if (s.robots[i].type == robot_type && s.robots[i].id == robot_id)
      {
        index = i;
      }
    }
    return index;
  }

  double ForwardSearch::dist(const std::vector<double> p1, const std::vector<double> p2)
  {
    if (p1.size() != p2.size())
    {
      std::cout << "Error! p1.size() != p2.size() for computing distance!\n";
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

  std::vector<std::vector<int>> cartesian_product(const std::vector<std::vector<int>> &v)
  {
    std::vector<std::vector<int>> s = {{}};
    for (const auto &u : v)
    {
      std::vector<std::vector<int>> r;
      for (const auto &x : s)
      {
        for (const auto &y : u)
        {
          r.push_back(x);
          r.back().push_back(y);
        }
      }
      s = std::move(r);
    }
    return s;
  }
}
