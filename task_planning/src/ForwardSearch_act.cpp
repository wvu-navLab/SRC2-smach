#include <task_planning/ForwardSearch.hpp>

namespace mac
{
  std::vector<std::vector<Action>> ForwardSearch::get_actions_all_robots(const State &s)
  {
    //get all possible actions for individual robots
    std::vector<std::vector<Action>> all_robots_actions;
    std::vector<std::vector<int>> all_robots_actions_indices;
    for (int i = 0; i < s.robots.size(); i++)
    {
      //all possible actions for robot i
      std::vector<Action> robot_actions = get_actions_robot(i, s);
      if (robot_actions.empty())
      {
        continue;
      }
      Action temp_a;
      // if (robot.type == SCOUT)
      // {
      //   temp_a.code =  (int)ACTION_EXCAVATOR_T::_planning;
      // }
      if (s.robots[i].type == EXCAVATOR)
      {
        temp_a.code = (int)ACTION_EXCAVATOR_T::_planning;
        temp_a.robot_type = s.robots[i].type;
      }
      if (s.robots[i].type == HAULER)
      {
        temp_a.code = (int)ACTION_HAULER_T::_planning;
        temp_a.robot_type = s.robots[i].type;
      }
      robot_actions.push_back(temp_a);
      all_robots_actions.push_back(robot_actions);

      //indices of actions need for cartesian product
      std::vector<int> robot_actions_indices(robot_actions.size());
      std::iota(robot_actions_indices.begin(), robot_actions_indices.end(), 0);
      all_robots_actions_indices.push_back(robot_actions_indices);
    }
    int combinations = 1;
    for (auto &a : all_robots_actions)
    {
      combinations *= a.size();
    }

    //get all joint actions from indices
    //std::cout << "get_actions_all_robots: getting cartesian product" << std::endl;
    std::vector<std::vector<int>> all_joints_actions_indices;
    all_joints_actions_indices = cartesian_product(all_robots_actions_indices);
    //std::cout << "get_actions_all_robots: number of joint actions (index type) " << all_joints_actions_indices.size() << std::endl;

    //populate joint actions from cartesian product
    std::vector<std::vector<Action>> all_joint_actions;
    for (int i = 0; i < all_joints_actions_indices.size(); i++)
    {
      std::vector<Action> joint_action;
      for (int j = 0; j < all_joints_actions_indices[i].size(); j++)
      {
        joint_action.push_back(all_robots_actions[j][all_joints_actions_indices[i][j]]);
      }
      all_joint_actions.push_back(joint_action);
    }
    //std::cout << "get_actions_all_robots: number of joint actions (action type) " << all_joint_actions.size() << std::endl;
    //std::cout << "sizey boi dirty: " << all_joint_actions.size() << std::endl;

    // delete actions where either excavators go to same volatile or haulers go to same volatile
    std::vector<std::vector<Action>> all_joint_actions_clean;
    for (const auto &ja : all_joint_actions)
    {
      std::vector<int> exc_vols;
      exc_vols.clear();
      std::vector<int> hau_vols;
      hau_vols.clear();
      bool is_bad_action = false;
      for (const auto &a : ja)
      {
        if (a.volatile_index == -1)
        {
          continue;
        }

        if (a.robot_type == mac::EXCAVATOR)
        {
          if (exc_vols.empty())
          {
            exc_vols.push_back(a.volatile_index);
            continue;
          }
          if (std::find(exc_vols.begin(), exc_vols.end(), a.volatile_index) != exc_vols.end()) //checks if a.volatile_index is in exc_vols
          {
            is_bad_action = true;
            break;
          }
          else
          {
            exc_vols.push_back(a.volatile_index);
          }
        }
        else if (a.robot_type == mac::HAULER)
        {
          if (hau_vols.empty())
          {
            hau_vols.push_back(a.volatile_index);
            continue;
          }
          if (std::find(hau_vols.begin(), hau_vols.end(), a.volatile_index) != hau_vols.end()) //checks if a.volatile_index is in hau_vols
          {
            is_bad_action = true;
            break;
          }
          else
          {
            hau_vols.push_back(a.volatile_index);
          }
        }
      }
      if (!is_bad_action)
        all_joint_actions_clean.push_back(ja);
    }

    //std::cout << "sizey boi clean: " << all_joint_actions_clean.size() << std::endl;

    //delete planning actions, and delete joints actions that are empty
    std::vector<std::vector<Action>> all_joint_actions_clean_clean;
    //std::cout << "PRINTING SET OF NEW JOINT ACTIONS ---------------" << std::endl;
    for (auto &ja : all_joint_actions_clean)
    {
      std::vector<Action> joint_action_clean_clean;
      for (auto &local_a : ja)
      {
        // if (robot.type == SCOUT && temp_a.code == (int)ACTION_SCOUT_T::_planning)
        // {
        //   temp_a.code =  (int)ACTION_EXCAVATOR_T::_planning;
        // }
        if (local_a.robot_type == EXCAVATOR && local_a.code == (int)ACTION_EXCAVATOR_T::_planning)
        {
          continue;
        }
        if (local_a.robot_type == HAULER && local_a.code == (int)ACTION_HAULER_T::_planning)
        {
          continue;
        }
        joint_action_clean_clean.push_back(local_a);
        //std::cout << "local a code " << local_a.code << ", type " << local_a.robot_type << ")" << std::endl;
      }
      if (!joint_action_clean_clean.empty())
      {
        all_joint_actions_clean_clean.push_back(joint_action_clean_clean);
        // std::cout << "New JOINT ACTION" << std::endl;
        // for (auto &act : joint_action_clean_clean)
        // {
        //   std::cout << "robot: (type " << act.robot_type << ",id " << act.id << ") | objective: " << act.code << " (" << act.objective.first << "," << act.objective.second << ")" << std::endl;
        // }
      }
    }
    //std::cout << "sizey boi_clean_clean: " << all_joint_actions_clean_clean.size() << std::endl;

    return all_joint_actions_clean_clean;
  }

  std::vector<Action> ForwardSearch::get_actions_robot(int robot_index, const State &s)
  {
    // std::cout << "GET ACTIONS ROBOT " << robot_index << std::endl;
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
      //actions = get_actions_hauler(robot_index, s);
    }

    //add actions for toggle_sleep = true
    // std::vector<Action> copy_actions = actions;
    // for (const auto &action : copy_actions)
    // {
    //   Action copy_action = action;
    //   copy_action.toggle_sleep = true;
    //   actions.push_back(copy_action);
    // }
    //std::cout << "get_actions_robot: " << actions.size() << " action size" << std::endl;

    return actions;
  }

  std::vector<Action> ForwardSearch::get_actions_scout(int robot_index, const State &s)
  {
    std::vector<Action> actions;
    Robot robot = s.robots[robot_index];

    //if task is in progress, do not return any actions
    if (robot.time_remaining > 1e-6)
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
    if (robot.time_remaining > 1e-6)
    {
      //std::cout << "get_actions_excavator: time remaining" << std::endl;
      actions.clear();
      return actions;
    }

    //action LOST
    if (!disable_lost_action_)
    {
      if (0 > planning_params_.max_unc || 0 < planning_params_.min_power) //replace 0 with e.g., trace of covariance etc...
      {
        Action a;
        a.objective = std::make_pair(0, 0); //may need changed
        a.robot_type = robot.type;
        a.id = robot.id;
        a.code = (int)ACTION_EXCAVATOR_T::_lost;
        a.volatile_index = -1;
        a.toggle_sleep = false;
        actions.push_back(a);
        return actions;
      }
    }

    //add indices of volatiles being pursued by other excavators to blacklist
    std::vector<int> vol_blacklist;
    for (const auto &r : s.robots)
    {
      if (r.type == EXCAVATOR)
      {
        if (r.volatile_index != -1) //-1 means excavator is not pursuing a volatile
        {
          vol_blacklist.push_back(r.volatile_index);
        }
      }
    }
    //std::cout << "get_actions_excavator: " << vol_blacklist.size() << " out of " << s.volatile_map.vol.size() << " blacklisted" << std::endl;

    //actions VOLATILES
    int volatile_index = 0; //incremented in for loop
    for (const auto &vol : s.volatile_map.vol)
    {
      //skip blacklisted volatiles
      bool is_vol_blacklisted = false;
      for (const auto vol_bl : vol_blacklist)
      {
        if (volatile_index == vol_bl)
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
        a.id = robot.id;
        a.code = (int)ACTION_EXCAVATOR_T::_volatile_handler;
        a.volatile_index = volatile_index;
        a.toggle_sleep = false;
        //a.action_time = dist({a.objective.first, a.objective.second},
        //                     {s.volatile_map.vol[volatile_index].position.point.x, s.volatile_map.vol[volatile_index].position.point.y}) /
        //                     planning_params_.max_v_excavator; //TODO: update later to appropriate time
        actions.push_back(a);
        //std::cout << "get_actions_excavator: volatile " << volatile_index << " action added" << std::endl;
      }
      volatile_index++;
    }

    //action PLANNING (i.e., wait action)
    if (!disable_planning_action_)
    {
      Action a;
      a.objective = std::make_pair(robot.odom.pose.pose.position.x,
                                   robot.odom.pose.pose.position.y);
      a.robot_type = robot.type;
      a.id = robot.id;
      a.code = (int)ACTION_EXCAVATOR_T::_planning;
      a.volatile_index = -1;
      a.toggle_sleep = false;
      //a.action_time = 30;
      actions.push_back(a);
      //std::cout << "get_actions_excavator: planning action added " << std::endl;
    }

    return actions;
  }

  std::vector<Action> ForwardSearch::get_actions_hauler(int robot_index, const State &s)
  {
    //std::cout << "GET ACTIONS HAULER, ROBOT " << robot_index << std::endl;

    std::vector<Action> actions;
    Robot robot = s.robots[robot_index];

    //if task is in progress, do not return any actions
    if (robot.time_remaining > 1e-6)
    {
      //std::cout << "get_actions_hauler: time remaining" << std::endl;

      actions.clear();
      return actions;
    }

    //action LOST
    if (!disable_lost_action_)
    {
      if (0 > planning_params_.max_unc || 100 < planning_params_.min_power) //replace 0 with e.g., trace of covariance etc...
      {
        Action a;
        a.objective = std::make_pair(0, 0); //may need changed...
        a.robot_type = robot.type;
        a.id = robot.id;
        a.code = (int)ACTION_HAULER_T::_lost;
        a.volatile_index = -1;
        a.toggle_sleep = false;
        actions.push_back(a);
        return actions;
      }
    }

    //only consider volatiles that are being pursued by excavators
    std::vector<int> vol_indices; //contains volatile indices excavators are approaching
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
    //std::cout << "get_actions_hauler: " << vol_indices.size() << " vol indices size" << std::endl;

    //add indices of volatiles being pursued by other hauler to blacklist
    std::vector<int> vol_blacklist; //contains volatiles indices that are blacklisted
    for (const auto &r : s.robots)
    {
      if (r.type == HAULER)
      {
        if (r.volatile_index != -1) //-1 means hauler is not pursuing a volatile
        {
          vol_blacklist.push_back(r.volatile_index);
        }
      }
    }

    //actions VOLATILES
    for (int i = 0; i < vol_indices.size(); i++)
    // for (const auto &vol_index : vol_indices)
    {
      //skip blacklisted volatiles
      bool is_vol_blacklisted = false;
      for (const auto vol_bl : vol_blacklist)
      {
        if (vol_indices[i] == vol_bl)
        {
          is_vol_blacklisted = true;
        }
      }
      if (is_vol_blacklisted)
      {
        continue;
      }

      Action a;
      // a.objective = std::make_pair(s.volatile_map.vol[vol_index].position.point.x,
      //                              s.volatile_map.vol[vol_index].position.point.y);
      a.objective = std::make_pair(s.volatile_map.vol[vol_indices[i]].position.point.x,
                                   s.volatile_map.vol[vol_indices[i]].position.point.y);
      a.robot_type = robot.type;
      a.id = robot.id;
      a.code = (int)ACTION_HAULER_T::_volatile_handler;
      // a.volatile_index = vol_index;
      a.volatile_index = vol_indices[i];
      a.toggle_sleep = false;
      //a.action_time = dist({a.objective.first, a.objective.second},
      //                       {s.volatile_map.vol[volatile_index].position.point.x, s.volatile_map.vol[volatile_index].position.point.y})/
      //                        planning_params_.max_v_hauler;
      actions.push_back(a);
      //std::cout << "get_actions_hauler: volatile " << vol_index << " action added" << std::endl;
    }

    //action PLANNING (i.e., wait action)
    //maybe add if here
    if (!disable_planning_action_)
    {
      Action a;
      a.objective = std::make_pair(robot.odom.pose.pose.position.x,
                                   robot.odom.pose.pose.position.y);
      a.robot_type = robot.type;
      a.id = robot.id;
      a.code = (int)ACTION_HAULER_T::_planning;
      a.volatile_index = -1;
      a.toggle_sleep = false;
      //a.action_time = 30;
      actions.push_back(a);
      //std::cout << "get_actions_hauler: planning action added " << std::endl;
    }

    //action DUMP (not handled by task planner)
    // if (robot.bucket_contents > 1e-6) //robot.current_task != (int)ACTION_HAULER_T::_volatile_handler
    // {
    //   if (robot.volatile_index == -1)
    //   {
    //     std::cout << "get_actions_hauler: Error! Invalid volatile index for selecting dumping action!" << std::endl;
    //   }
    //   else
    //   {
    //     Action a;
    //     a.objective = std::make_pair(0, 0);
    //     a.robot_type = robot.type;
    //     a.id = robot.id;
    //     a.code = (int)ACTION_HAULER_T::_hauler_dumping;
    //     a.toggle_sleep = false;
    //     actions.push_back(a);
    //     //std::cout << "get_actions_hauler: hauler conents are " << robot.bucket_contents << " of 1" << std::endl;
    //     //std::cout << "get_actions_hauler: dumping action added " << std::endl;
    //   }
    // }
    return actions;
  }

  std::vector<std::vector<Action>> ForwardSearch::get_sequence_of_joint_actions(int depth, int layer_index)
  {
    std::vector<std::vector<Action>> seq_joint_actions;

    Vertex leaf = tree_[depth][layer_index];
    while (leaf.depth != 0)
    {
      seq_joint_actions.push_back(leaf.joint_action);
      leaf = tree_[leaf.depth - 1][leaf.parent_layer_index];
    }

    return seq_joint_actions;
  }

  std::vector<std::vector<std::vector<Action>>> ForwardSearch::get_all_sequences_of_joint_actions()
  {
    std::vector<std::vector<std::vector<Action>>> all_joint_actions;

    for (const auto &leaf : tree_[tree_.size() - 1])
    {
      std::vector<std::vector<Action>> joint_actions = get_sequence_of_joint_actions(leaf.depth, leaf.layer_index);
      all_joint_actions.push_back(joint_actions);
    }
    return all_joint_actions;
  }

  std::vector<std::vector<Action>> ForwardSearch::get_best_sequence_of_joint_actions()
  {
    //select leaf with minimum cost (only works if problem is deterministic)
    int min_layer_index = -1;
    int min_depth = -1;
    double min_total_cost = 1e9;
    for (auto &v : tree_[tree_.size() - 1])
    {
      if (v.total_cost < min_total_cost)
      {
        min_layer_index = v.layer_index;
        min_depth = v.depth;
        min_total_cost = v.total_cost;
      }
    }

    if (min_layer_index == -1 || min_depth == -1)
    {
      std::cout << "get_best_sequence_of_joint_actions: No leaf selected! No joint action returned." << std::endl;
      std::vector<std::vector<Action>> empty_seq_joint_action;
      return empty_seq_joint_action;
    }

    return get_sequence_of_joint_actions(min_depth, min_layer_index);
  }
}