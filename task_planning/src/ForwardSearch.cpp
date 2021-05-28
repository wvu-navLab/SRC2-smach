#include <task_planning/ForwardSearch.hpp>

namespace mac{

/////////////////////////////////////////////////////////////////////
/***************************CONSTRUCTORS****************************/
/////////////////////////////////////////////////////////////////////

ForwardSearch::ForwardSearch() {}

ForwardSearch::ForwardSearch(const CostFunction   & cost_function,
                             const PlanningParams & planning_params)
    : cost_function_(cost_function),
      robots_(robots),
      planning_params_(planning_params) {}

/////////////////////////////////////////////////////////////////////
/***************************GETTERS*********************************/
/////////////////////////////////////////////////////////////////////

void ForwardSearch::get_state(const std::vector<Robot>        & robots,
                              const volatile_map::VolatileMap & volatile_map,
                              const ros::Time                 & time)
{
  state_.robots = robots;
  state_.volatile_map_ = volatile_map;
  state_.time_ = time;
}


/////////////////////////////////////////////////////////////////////
/**************************CORE FUNCTIONALITY***********************/
/////////////////////////////////////////////////////////////////////
Action ForwardSearch::plan(const State & s)
{
  state_ = s;

  int depth = 0;
  while (depth < n-1)
  {

    std::vector<Vertex> temp;
    int i = 0;
    for(auto & leaf:tree[depth])
    {
      std::vection<int> a = get_actions(leaf);
      if (!a.empty())
      {
        std::vector<Vertex> vert = expand(leaf,a);
        for (auto & v:vert)
        {
          leaf.children.push_back(i);
          //TODO: check this updates the leaf in the tree
          v.index = i;
          v.parent = leaf.index;
          v.depth = depth+1;
          temp.push_back(v);
          ++i;
        }
      }
    }

    if (!temp.empty())
    {
      tree.push_back(temp);
    }

    temp.clear();
    ++depth;
  }

  return get_policy();

}

bool ForwardSearch::reinit()
{

}

std::vector<Vertex> ForwardSearch::expand(const Vertex              & v, 
                                          const std::vector<Action> & actions)
{
  std::vector<Vertex> V;
  for (auto &a:actions)
  {
      Vertex tempv = propagate(v,a);

      V.push_back(tempv);
  }
  return V;
}


State propagate(const State  & s, 
                const Action & a)
{
  int robot_ind get_robot_index(a.robot_type, a.id); //NOTE what if there are multiple
  s.robots[robot_ind].status = 0;
  s.robots[robot_ind].current_task = a.code;
  s.robots[robot_ind].plan.clear();
  geometry_msgs::PointStamped temp_msg;
  temp_msg.point.x = a.objective.x;
  temp_msg.point.y = a.objective.y;
  s.robots[robot_ind].plan.push_back(temp_msg);
  s.robots[robot_ind].volatile_index = a.volatile_index;

  // Find time remaining for each robot's task
  std::vector<double> time_remaining;
  for (auto &robot:s.robots)
  {
    time_remaining.push_back(simulate_time_remaining(*robot));
  }
  // Find min time remaining
  double min_time_idx = std::min_element(time_remaining.begin(),time_remaining.end()) - time_remaining.begin();
  double min_time = time_remaining[min_time_idx];
  // Update volatile completion if necessary
  temp_vol_ind = s.robots[min_time_index].volatile_index;
  if (s.robots[min_time_index] == ACTION_HAULER_T::_volatile_handler)
  {
    s.volatile_map.vol[temp_vol_ind].collect = true;
  }
  // propagate completion for each robot
  for (auto &robot:s.robots)
  {
    simulate_time_step(*robot, min_time);
  }

  return s;
}

std::vector<Action> ForwardSearch::get_actions_all_robots(const State & s)
{
    std::vector<Action> actions;
    for (int i; i < state_.robots.size(); ++i)
    {
        std::vector<Action> temp;
        temp = get_actions_robot(i,s);
        for (auto &item:temp) actions.push_back(item);
    }
}

std::vector<Action> ForwardSearch::get_actions_robot(int robot_index, const State & s)
{
  std::vector<Action> actions;

  //get actions for SCOUT
  if(robot.type == SCOUT) {
    actions = get_actions_scout(robot_index, s);
  }

  //get actions for EXCAVATOR
  if(robot.type == EXCAVATOR) {
    actions = get_actions_excavator(robot_index, s);
  }

  //get actions for HAULER
  if(robot.type == HAULER) {
    actions = get_actions_hauler(robot_index, s);
  }

  //add actions for toggle_sleep = true
  std::vector<Action> copy_actions = actions;
  for(const auto & action : copy_actions) {
      action.toggle_sleep = true;
      actions.push_back(action);
  }

  return actions;
}

std::vector<Action> ForwardSearch::get_actions_scout(int robot_index, const State & s) {
  std::vector<Action> actions;
  Robot robot = s.robots[robot_index];
  
  //if task is in progress, do not return any actions
  if(robot.status < 1) {
    actions.clear();
    return actions; 
  }

  //TODO: the scout is not currently considered in this planner
  actions.clear();
  return actions;
}

std::vector<Action> ForwardSearch::get_actions_excavator(int robot_index, const State & s) {
  std::vector<Action> actions;
  Robot robot = s.robots[robot_index];

  //if task is in progress, do not return any actions
  if(robot.status < 1) { 
    actions.clear();
    return actions;
  }

  //add indices of volatiles being pursued by other excavators to blacklist
  std::vector<int> vol_blacklist;
  for(const auto & r : s.robots) {
    if(r.type!=EXCAVATOR) {
      continue;
    }
    if(r.volatile_index!=-1) {
      vol_blacklist.push_back(r.volatile_index); 
    }
  }

  //actions VOLATILES
  int volatile_index=0;
  for(const auto & vol : volatile_map.vol) {
    //skip blacklisted volatiles
    bool is_vol_blacklisted = false;
    for(const auto vol_bl : vol_blacklist) {
      if(vol==vol_bl) {
        is_vol_blacklisted = true;
      }
    }
    if(is_vol_blacklisted) {
      continue;
    }

    //add actions for volatiles not in blacklist
    if(!vol.collected) {
      Action a;
      a.objective = std::make_pair(vol.position.point.x, 
                                   vol.position.point.y);
      a.robot_type = robot.type;
      a.id = robot_index;
      a.code = ACTION_EXCAVATOR_T::_volatile_handler;
      a.volatile_index = volatile_index;
      a.toggle_sleep = false;
      actions.push_back(a);
    }
    volatile_index++;
  }

  //action LOST
  Action a;
  a.objective = std::make_pair(0,0);
  a.robot_type = robot.type;
  a.id = robot_index;
  a.code = ACTION_EXCAVATOR_T::_lost;
  a.volatile_index = -1;
  a.toggle_sleep = false;
  actions.push_back(a);

  //action PLANNING (i.e., wait action)
  Action a;
  a.objective = std::make_pair(robot.odom.pose.pose.position.x,
                               robot.odom.pose.pose.position.y);
  a.robot_type = robot.type;
  a.id = robot_index;
  a.code = ACTION_EXCAVATOR_T::_planning;
  a.volatile_index = -1;
  a.toggle_sleep = false;
  actions.push_back(a);
}

std::vector<Action> ForwardSearch::get_actions_hauler(int robot_index, const State & s) {
  std::vector<Action> actions;
  Robot robot = s.robots[robot_index];

  //if task is in progress, do not return any actions
  if(robot.status < 1) { 
    actions.clear();
    return actions;
  }

  //only consider volatiles that are being pursued by excavators
  std::vector<int> vol_indices;
  for(const auto & r : s.robots) {
    if(r.type == EXCAVATOR) {
      if(r.volatile_index==-1) {
        continue;
      }
      else {
        vol_indices.push_back(r.volatile_index);
      }
    }
  }

  //actions VOLATILES
  for(const auto & vol_index : vol_indices) {
    Action a;
    a.objective = std::make_pair(volatile_map.vol[vol_index].position.point.x, 
                                 volatile_map.vol[vol_index].position.point.y);
    a.robot_type = robot.type;
    a.id = robot_index;
    a.code = ACTION_HAULER_T::_volatile_handler;
    a.volatile_index = vol_index;
    a.toggle_sleep = false;
    actions.push_back(a);
    volatile_index++;
  }

  //action LOST
  Action a;
  a.objective = std::make_pair(0,0);
  a.robot_type = robot.type;
  a.id = robot_index;
  a.code = ACTION_HAULER_T::_lost;
  a.volatile_index = -1;
  a.toggle_sleep = false;
  actions.push_back(a);

  //action PLANNING (i.e., wait action)
  Action a;
  a.objective = std::make_pair(robot.odom.pose.pose.position.x,
                               robot.odom.pose.pose.position.y);
  a.robot_type = robot.type;
  a.id = robot_index;
  a.code = ACTION_HAULER_T::_planning;
  a.volatile_index = -1;
  a.toggle_sleep = false;
  actions.push_back(a);

  //action DUMP
  Action a;
  a.objective = std::make_pair(0,0);
  a.robot_type = robot.type;
  a.id = robot_index;
  a.code = ACTION_HAULER_T::_dump;
  a.volatile_index = -1;
  a.toggle_sleep = false;
  actions.push_back(a);
}

int ForwardSearch::get_policy()
{
  int max_ind = 0;
  doublt max_cost = -5e6;
  for (auto & v:tree[tree.size()-1])
  {
    if (max_cost < v.cost)
    {
      max_ind = v.index;
      max_cost = v.cost;
    }
  }

  return get_parent(max_ind, tree.size());
}

int ForwardSearch::get_parent(int ind_v,
                              int depth)
{
  if (depth == 1)
  {
    return ind;
  } else
  {
    get_parent(tree[depth][ind_v].parent,depth-1);
  }
}

/////////////////////////////////////////////////////////////////////
/***************************UTILITIES*******************************/
/////////////////////////////////////////////////////////////////////

double ForwardSearch::simulate_time_remaining(Robot *robot)
{
  //status to timestep
  
  // switch case based on code
    // convert status remaining to time (s)
  switch (robot.code)
  {
    case ACTION_HAULER_T::_planning:
      break;
    case ACTION_HAULER_T::_volatile_handler:
      break;
    case ACTION_HAULER_T::_lost:
      break;
    case ACTION_HAULER_T::_hauler_dumping:
      break;
    default:  
  }

}

void ForwardSearch::simulate_time_step(Robot *robot, 
                                       double status_increment)
{
  // get veolcity from toggle
  
  //propagate position
    // velocity and distance
  //propagate status 
    // switch case based on code
        //convert timestep to status increment
  //propagate Uncertainty
    // distance stochastic model
  //propagate power 
    // motion and power consumption/supply
  switch (robot->code)
  {
    case ACTION_HAULER_T::_planning:
      break;
    case ACTION_HAULER_T::_volatile_handler:
      break;
    case ACTION_HAULER_T::_lost:
      break;
    case ACTION_HAULER_T::_hauler_dumping:
      break;
    default:  
  }
}

int ForwardSearch::get_robot_index(int robot_type, 
                                   int robot_id)
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