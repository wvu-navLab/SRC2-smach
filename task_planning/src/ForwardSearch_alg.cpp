#include <task_planning/ForwardSearch.hpp>

namespace mac
{

  ForwardSearch::ForwardSearch(CostFunction cost_function,
                               PlanningParams planning_params)
      : cost_function_(cost_function),
        planning_params_(planning_params)
  {
    // initialize the random number generator with time-dependent seed
    uint64_t timeSeed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    std::seed_seq ss{uint32_t(timeSeed & 0xffffffff), uint32_t(timeSeed >> 32)};
    rng_.seed(ss);
    // initialize a uniform distribution between 0 and 1
    unif_ = *(new std::uniform_real_distribution<double>(0, 1));
  }

  std::vector<Action> ForwardSearch::plan(const State &s)
  {
    // std::cout << "PLAN" << std::endl;
    std::cout << "plan: running forward search... " << std::endl;
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
    double accum_time = 0;
    double time_diff = 0;
    ros::Rate rate(1000);
    rate.sleep();
    double prev_time = ros::Time::now().toSec();
    while (depth < planning_params_.max_depth - 1)
    {
      std::cout << "plan: constructing layer " << depth + 1 << std::endl;

      std::vector<Vertex> layer;
      for (auto &leaf : tree_[depth])
      {
        //std::cout << "plan: getting actions all robots..." << std::endl;
        std::vector<std::vector<Action>> joint_actions = get_actions_all_robots(leaf.state);
        //std::cout << "plan:" << joint_actions.size() << " joint actions returned" << std::endl;
        //rate.sleep();
        // print_sequence_of_joint_actions(joint_actions);
        for (auto &joint_action : joint_actions)
        {
          //std::cout << "plan: creating vertex " << layer.size() << " at layer " << tree_.size() << std::endl;
          // if (tree_.size() == 4)
          // {
          // for (auto &layer : tree_)
          //   std::cout << layer.size() << std::endl;
          // exit(1);
          // }
          // if ( depth > 0 && tree_[depth - 1].size() > 5000)
          //   exit(1);
          Vertex v_new;
          v_new.state = propagate(leaf.state, joint_action);
          // std::cout << "TIME REMAINING NEW STATE" << std::endl;
          // for (auto &robot : v_new.state.robots)
          //   std::cout << robot.time_remaining << " | ";
          // std::cout << std::endl;
          v_new.joint_action = joint_action;
          v_new.cost = cost_function_.compute_cost(leaf.state, joint_action, v_new.state);
          v_new.total_cost = leaf.total_cost + v_new.cost;
          v_new.parent_layer_index = leaf.layer_index;
          v_new.depth = tree_.size();
          v_new.layer_index = layer.size();
          v_new.children.clear();
          layer.push_back(v_new);
        }
        // std::cout << "plan: end layer push back" << std::endl;
      }

      //accumulate time
      double curr_time = ros::Time::now().toSec();
      time_diff = curr_time - prev_time;
      prev_time = curr_time;
      accum_time = accum_time + time_diff;
      //std::cout << "plan: accum_time = " << accum_time << std::endl;

      if (accum_time + 2*time_diff > planning_params_.max_time)
      {
        std::cout << "plan: time exceeded. Terminating constructing tree." << std::endl;
        break;
      }

      //std::cout << "plan: constructing layer complete" << std::endl;
      if (!layer.empty())
      {
        tree_.push_back(layer);
      }
      else
      {
        std::cout << "No possible joint actions at current depth. Terminating constructing tree." << std::endl;
        break;
      }

      layer.clear();
      ++depth;
    }

    return get_policy();
  }

  State ForwardSearch::propagate(const State &s,
                                 const std::vector<Action> &joint_action)
  {
    // std::cout << "PROPAGATE" << std::endl;

    State s_copy = s;
    //std::cout << "propagate: state" << std::endl;

    // std::cout << "TIME REMAINING BEFORE" << std::endl;
    // for (auto &robot : s_copy.robots)
    //   std::cout << robot.time_remaining << " | ";
    // std::cout << std::endl;
    for (auto &a : joint_action)
    {
      int robot_ind = get_robot_index(s_copy, a.robot_type, a.id); //NOTE what if there are multiple
                                                                   //std::cout << "propagate: robot index received: " << robot_ind+1 << " of " << s_copy.robots.size() << std::endl;
      //std::cout << "current task: " << a.code <<  std::endl;
      s_copy.robots[robot_ind].current_task = a.code;
      s_copy.robots[robot_ind].plan.clear();
      geometry_msgs::PointStamped temp_msg;
      temp_msg.point.x = a.objective.first;
      temp_msg.point.y = a.objective.second;
      //std::cout << "objective: (" << a.objective.first << "," << a.objective.second << ")" <<  std::endl;

      s_copy.robots[robot_ind].plan.push_back(temp_msg);
      s_copy.robots[robot_ind].toggle_sleep = a.toggle_sleep;
      s_copy.robots[robot_ind].volatile_index = a.volatile_index;          //this changes later
      s_copy.robots[robot_ind].time_remaining = action_to_time(s_copy, a); //this change later

      //std::cout << "propagate: action a" << std::endl;
    }
    // std::cout << "WAIII" << std::endl;

    //std::cout << "propagate: actions added" << std::endl;
    std::vector<double> time_remaining_vec;
    for (auto &robot : s_copy.robots)
      time_remaining_vec.push_back(robot.time_remaining);

    // Find time remaining for each robot's task
    // = actions_to_time(s_copy, joint_action);
    //std::cout << "propagate: sorting action timesteps" << std::endl;

    // Find min time remaining
    //int t_min_idx = std::min_element(time_remaining_vec.begin(), time_remaining_vec.end()) - time_remaining_vec.begin();

    double t_min = 1e9;
    double t_min_idx = -1;
    int t_counter = 0;
    for (const auto &t : time_remaining_vec)
    {
      if (t < 1e-6)
      {
        continue;
      }
      if (t < t_min)
      {
        t_min = t;
        t_min_idx = t_counter;
      }
      ++t_counter;
    }
    if (t_min_idx == -1)
    {
      std::cout << "All robots have time remaining < 1e-6" << std::endl;
      exit(1);
    }

    //std::cout << "propagate: time remaining size" << time_remaining_vec.size() << std::endl;
    //double t_min = time_remaining_vec[t_min_idx];
    // std::cout << "TIME REMAINING AFTER" << std::endl;
    // for (auto &t : time_remaining_vec)
    //   std::cout << t << " | ";
    // std::cout << std::endl;

    // Update volatile completion if necessary
    int temp_vol_ind = s_copy.robots[t_min_idx].volatile_index;

    //std::cout << "propagate: propagating time step" << std::endl;

    // Need to come back and change this to hauler dumping... For now we are only handling "Excavation"

    // handles volatile collect excavator
    if (s_copy.robots[t_min_idx].type == mac::EXCAVATOR && s_copy.robots[t_min_idx].current_task == (int)ACTION_EXCAVATOR_T::_volatile_handler)
    {
      //check if hauler assigned to excavator
      bool is_there_hauler = false;
      for (const auto &r : s_copy.robots)
      {
        if (r.type == HAULER)
        {
          if (r.volatile_index == s_copy.robots[t_min_idx].volatile_index)
          {
            is_there_hauler = true;
          }
        }
      }
      if (is_there_hauler)
      {
        s_copy.volatile_map.vol[temp_vol_ind].collected = true;
        s_copy.robots[t_min_idx].volatile_index = -1;
        s_copy.robots[t_min_idx].current_task = (int)ACTION_EXCAVATOR_T::_planning;
        //std::cout << "Yeet got a vol" << std::endl;
      }
      else
      {
        s_copy.robots[t_min_idx].time_remaining += 30;
      }
    }
    //handles volatile collected hauler
    if (s_copy.robots[t_min_idx].type == mac::HAULER && s_copy.robots[t_min_idx].current_task == (int)ACTION_HAULER_T::_volatile_handler)
    {
      //s_copy.robots[t_min_idx].bucket_contents = 0;
      //std::cout << "s_copy.robots[t_min_idx].volatile_index = " << s_copy.robots[t_min_idx].volatile_index << std::endl;
      if (s_copy.robots[t_min_idx].volatile_index == -1)
      {
        std::cout << "Error! propagate:Invalid volatile index for hauler volatile_handling!" << std::endl;
      }
      //collected should be true already, or something is wrong
      s_copy.volatile_map.vol[s_copy.robots[t_min_idx].volatile_index].dumped = true;
      s_copy.robots[t_min_idx].volatile_index = -1;
      s_copy.robots[t_min_idx].current_task = (int)ACTION_HAULER_T::_planning;
      s_copy.robots[t_min_idx].odom.pose.pose.position.x = 0;
      s_copy.robots[t_min_idx].odom.pose.pose.position.y = 0;
    }

    if (s_copy.robots[t_min_idx].current_task ==(int)ACTION_SCOUT_T::_volatile_handler ||
        s_copy.robots[t_min_idx].current_task ==(int)ACTION_EXCAVATOR_T::_volatile_handler ||
        s_copy.robots[t_min_idx].current_task ==(int)ACTION_HAULER_T::_volatile_handler)
    {
        s_copy.robots[t_min_idx].power = 100;
        s_copy.robots[t_min_idx].odom.pose.covariance = {0,0,0,0,0,0,
                                                         0,0,0,0,0,0,
                                                         0,0,0,0,0,0,
                                                         0,0,0,0,0,0,
                                                         0,0,0,0,0,0,
                                                         0,0,0,0,0,0};
    }

    for (auto &robot : s_copy.robots)
    {
      simulate_time_step(robot, t_min);
    }
    s_copy.time_elapsed += t_min;
    // std::cout << "propagate: after simulate time step" << std::endl;
    // for (auto &robot : s_copy.robots)
    // {
    //   if (robot.type == mac::HAULER && robot.volatile_index == temp_vol_ind)
    //   {
    //     robot.bucket_contents = 1;
    //   }
    // }
    // //if (s_copy.robots[temp_vol_ind].volatile_index)
    // // propagate completion for each robot

    // std::cout << "TIME REMAINING AFTER AFTER" << std::endl;
    // for (auto &robot : s_copy.robots)
    //   std::cout << robot.time_remaining << " | ";
    // std::cout << std::endl;
    return s_copy;
  }

  std::vector<Action> ForwardSearch::get_policy()
  {
    // std::cout << "GET POLICY" << std::endl;
    // std::cout << "get_policy: depth of tree = " << tree_.size() << std::endl;
    // std::cout << "get_policy: size of final layer = " << tree_[tree_.size() - 1].size() << std::endl;

    //select leaf with minimum cost (only works if problem is deterministic)
    int min_layer_index = -1;
    int min_depth = -1;
    double min_total_cost = 1e9;
    for (auto &v : tree_[tree_.size() - 1])
    {
      //std::cout << "get_policy: v.total_cost " << v.total_cost << std::endl;
      if (v.total_cost < min_total_cost)
      {
        min_layer_index = v.layer_index;
        min_depth = v.depth;
        min_total_cost = v.total_cost;
        // std::cout << "get_policy: min total cost " << min_total_cost << std::endl;
      }
    }

    if (min_layer_index == -1 || min_depth == -1)
    {
      std::cout << "get_policy: No leaf selected in get_policy! No joint action returned." << std::endl;
      std::vector<Action> empty_joint_action;
      return empty_joint_action;
    }

    //find optimal joint action given selected leaf
    std::vector<Action> optimal_joint_action;
    bool is_action_valid = false;
    Vertex leaf = tree_[min_depth][min_layer_index];

    while (leaf.depth != 0)
    {
      //std::cout << "///////// DEPTH: " << leaf.depth << " /////////" << std::endl;
      // print_joint_action(leaf.joint_action);
      optimal_joint_action = leaf.joint_action;
      is_action_valid = true;
      leaf = tree_[leaf.depth - 1][leaf.parent_layer_index];
    }

    if (!is_action_valid)
    {
      std::cout << "get_policy: Error! No valid actions selected! Is the tree a depth of 0?." << std::endl;
    }
    // for (auto & leaf: tree_[tree_.size()-1])
    // {
    //   print_volatile_map(leaf.state.volatile_map);
    //   std::cout << "cost " << leaf.total_cost << std::endl;
    // }
    //std::cout << "-----------BEST LEAF----------" << std::endl;
    // print_volatile_map(tree_[min_depth][min_layer_index].state.volatile_map);
    //std::cout << "cost " << tree_[min_depth][min_layer_index].total_cost << std::endl;

    // std::vector<std::vector<mac::Action>> best_seq_of_joint_actions;
    // best_seq_of_joint_actions = this->get_best_sequence_of_joint_actions();
    // print_sequence_of_joint_actions(best_seq_of_joint_actions);

    // std::vector<Action> new_joint_action = optimal_joint_action;
    // for (auto & act: optimal_joint_action)
    // {
    //   //riv
    // }

    // return optimal_joint_action;
  }
}