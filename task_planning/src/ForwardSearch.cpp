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
  // we don't have time 
  // we do have state, objective
  // compute initial time remaining and add to state
  int depth = 0;
  while (depth < n-1)
  {
    std::vector<Vertex> temp;
    int i = 0;
    for(auto & leaf : tree[depth])
    {
      std::vector<Action> actions = get_actions_all_robots(leaf.s);
      if (!actions.empty())
      {
        std::vector<Vertex> vert = expand(leaf,actions);
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
  
  //for each combination of actions 
  // Vertex tempv = propagate(v, actions);
  // V.push_back(tempv);

  return V;
}


State propagate(const State  & s, 
                const std::vector<Action> & actions)
{

  //foreach action
  int robot_ind get_robot_index(a.robot_type, a.id); //NOTE what if there are multiple
  s.robots[robot_ind].status = 0;
  s.robots[robot_ind].current_task = a.code;
  s.robots[robot_ind].plan.clear();
  geometry_msgs::PointStamped temp_msg;
  temp_msg.point.x = a.objective.x;
  temp_msg.point.y = a.objective.y;
  s.toggle_sleep = a.toggle_sleep;
  s.robots[robot_ind].plan.push_back(temp_msg);
  s.robots[robot_ind].volatile_index = a.volatile_index;
  //--------------------------------------------------------------------------------
  //compute time remaining for new action --> action_to_time function
  // we're taking it on faith that we may get several immediate updates if more than one robot requires 

  //find shortest time

  // simulate timestep forall robots and return state --> time to everything else
  //--------------------------------------------------------------------------------

  //probably not doing this anymore ------------------------------------------------
  // // Find time remaining for each robot's task
  // std::vector<double> time_remaining;
  // for (auto &robot:s.robots)
  // {
  //   time_remaining.push_back(simulate_time_remaining(*robot));
  // }
  // // Find min time remaining
  // double min_time_idx = std::min_element(time_remaining.begin(),time_remaining.end()) - time_remaining.begin();
  // double min_time = time_remaining[min_time_idx];
  // // Update volatile completion if necessary
  // temp_vol_ind = s.robots[min_time_index].volatile_index;
  // if (s.robots[min_time_index] == ACTION_HAULER_T::_volatile_handler)
  // {
  //   s.volatile_map.vol[temp_vol_ind].collect = true;
  // }
  // // propagate completion for each robot
  // for (auto &robot:s.robots)
  // {
  //   simulate_time_step(*robot, min_time);
  // }

  return s;
}

std::vector<std::vector<Action>> ForwardSearch::get_actions_all_robots(const State & s)
{
    //get all possible actions for individual robots
    std::vector<std::vector<Action>> all_robots_actions;
    std::vector<std::vector<int>> all_robots_actions_indices;
    for(int i=0; i<state_.robots.size(); i++){
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
    for (int i=0; i<all_joints_actions_indices.size(); i++)
    {
      std::vector<Action> joint_action;
      for (int j=0; j<all_joints_actions_indices[i].size(); j++) {
        joint_action.push_back(all_robots_actions[j][all_joints_actions_indices[i][j]]);
      }
    }
    return joint_action;
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
  if (robot.time_remaining < 1e-6)
  {
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
  if (robot.time_remaining < 1e-6)
  {
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
  if (robot.time_remaining < 1e-6)
  {
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
// populates time remaing for tasks
std::vector<double> ForwardSearch::action_to_time(const State  & s, 
                                     const std::vector<Action> & a)
{
  //forall push_back(action_to_time(index, s, a))
}

double ForwardSearch::action_to_time(int robot_index,
                                     const State  & s, 
                                     const Action & a)
{
  //get robot
  Robot robot = s.robots[robot_index];

  if (robot.type == SCOUT)
  {
    switch (robot.code)
    {
      case ACTION_SCOUT_T::_planning:
        return planning_params_.wait_time; 
        break;
      case ACTION_SCOUT_T::_volatile_handler:
        double traverse_time = get_traverse_time(); // this will be based on dist to next next waypoint
        double handling_time = get_handling_time(); // this will fraction of edges where there is volatile * time it takes to handle volatile... likely be a constant
        return traverse_time + handling_time;
        break;
      case ACTION_SCOUT_T::_lost:
        double traverse_time = get_traverse_time(); // time to get to "home base waypoint" --> this waypoint may not be fixed.... may need a function to compute that
        double homing_time = get_homing_time(); // time to charge and localize with base station "home base waypoint"
        return traverse_time + homing_time;
        break;
      default:
    }
  }

  if (robot.type == EXCAVATOR)
  {
    switch (robot.code)
    {
      case ACTION_EXCAVATOR_T::_planning:
        return planning_params_.wait_time;
        break;
      case ACTION_EXCAVATOR_T::_volatile_handler:
        double traverse_time = get_traverse_time(); //see above
        double handling_time = get_handling_time(); // excavation time estimate... who knows... bernardo help, plzzzz!!!
        return traverse_time + handling_time;
        break;
      case ACTION_EXCAVATOR_T::_lost:
        double traverse_time = get_traverse_time(); //see above
        double homing_time = get_homing_time();     //see above
        return traverse_time + homing_time;
        break;
      default:
    }
  }

  if (robot.type == HAULER)
  {
    switch (robot.code)
    {
      case ACTION_HAULER_T::_planning:
        return planning_params_.wait_time;
        break;
      case ACTION_HAULER_T::_volatile_handler:
        double traverse_time = get_traverse_time(); //see above
        double handling_time = get_handling_time(); //see above
        return traverse_time + handling_time;
        break;
      case ACTION_HAULER_T::_lost:
        double traverse_time = get_traverse_time(); //see above
        double homing_time = get_homing_time();     //see above
        return traverse_time + homing_time;
        break;
      case ACTION_HAULER_T::_hauler_dumping:
        double traverse_time = get_traverse_time(); //see above
        double dumping_time = get_dumping_time();   // time from "dumping waypoint" to alignment with base + dump
        return traverse_time + dumping_time;
        break;
      default:
    }
  }
}

double ForwardSearch::get_traverse_time(Robot &robot)
{
  switch (robot.type)
  {
    double v;
    case mac::SCOUT:
      v = max_v_scout;
      break;
    case mac::EXCAVATOR:
      v = max_v_excavator;
      break;
    case mac::HAULER:
      v = max_v_hauler;
      break;
    default:
      ROS_ERROR("Invalid robot type: get_traverse_time");
  }

  if toggle_sleep)  v *= 0.75;

  std::vector<double> p1, p2;
  p1.push_back(robot.odom.pose.pose.position.x);
  p1.push_back(robot.odom.pose.pose.position.y);
  p2.push_back(robot.plan[0].point.x);
  p2.push_back(robot.plan[0].point.y);
  
  return this->dist(p1,p2)/v;
  
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
  this->time_to_power(robot,time);
  // simulate position and uncertainty
  this->time_to_motion(robot,time);
  
}

double ForwardSearch::time_to_power(Robot &robot,
                                    double time)
{
  if (robot.type == SCOUT)
  {
    switch (robot->code)
    {
    case ACTION_SCOUT_T::_planning:
      for (int i = 0; i < planning_params_.power_rates.size(); ++i)
        robot.power -= planning_params_.planning_scout_power_weights[i]*planning_params_.power_rates[i]*time;
      return;
    case ACTION_SCOUT_T::_volatile_handler:
      for (int i = 0; i < planning_params_.power_rates.size(); ++i)
        robot.power -= planning_params_.vol_handle_scout_power_weights[i]*planning_params_.power_rates[i]*time;
      return; 
    case ACTION_SCOUT_T::_lost:
      for (int i = 0; i < planning_params_.power_rates.size(); ++i)
        robot.power -= planning_params_.lost_scout_power_weights[i]*planning_params_.power_rates[i]*time;
      return; 
    default:
       ROS_ERROR("Invalid robot code: time_to_power");
  }

  if (robot.type == EXCAVATOR)
  {
    switch (robot->code)
    {
    case ACTION_EXCAVATOR_T::_planning:
      for (int i = 0; i < planning_params_.power_rates.size(); ++i)
        robot.power -= planning_params_.planning_excavator_power_weights[i]*planning_params_.power_rates[i]*time;
      return;
    case ACTION_EXCAVATOR_T::_volatile_handler:
      for (int i = 0; i < planning_params_.power_rates.size(); ++i)
        robot.power -= planning_params_.vol_handle_excavator_power_weights[i]*planning_params_.power_rates[i]*time;
      return; 
    case ACTION_EXCAVATOR_T::_lost:
      for (int i = 0; i < planning_params_.power_rates.size(); ++i)
        robot.power -= planning_params_.lost_excavator_power_weights[i]*planning_params_.power_rates[i]*time;
      return; 
    default: 
      ROS_ERROR("Invalid robot code: time_to_power");

  }

  if (robot.type == HAULER)
  {
    switch (robot->code)
    {
    case ACTION_HAULER_T::_planning:
      for (int i = 0; i < planning_params_.power_rates.size(); ++i)
        robot.power -= planning_params_.planning_hauler_power_weights[i]*planning_params_.power_rates[i]*time;
      return;
    case ACTION_HAULER_T::_volatile_handler:
      for (int i = 0; i < planning_params_.power_rates.size(); ++i)
        robot.power -= planning_params_.vol_handle_hauler_power_weights[i]*planning_params_.power_rates[i]*time;
      return; 
    case ACTION_HAULER_T::_lost:
      for (int i = 0; i < planning_params_.power_rates.size(); ++i)
        robot.power -= planning_params_.lost_hauler_power_weights[i]*planning_params_.power_rates[i]*time;
      return; 
    default: 
      ROS_ERROR("Invalid robot code: time_to_power");

  }
}

double ForwardSearch::time_to_motion(Robot &robot,
                                     double time)
{
  // if position differs from target increment in direction, else do nothing
  switch (robot.type)
  {
    double v;
    case mac::SCOUT:
      v = max_v_scout;
      break;
    case mac::EXCAVATOR:
      v = max_v_excavator;
      break;
    case mac::HAULER:
      v = max_v_hauler;
      break;
    default:
      ROS_ERROR("Invalid robot type: get_traverse_time");
  }

  if toggle_sleep)  v *= 0.75;

  std::vector<double> p1, p2;
  p1.push_back(robot.odom.pose.pose.position.x);
  p1.push_back(robot.odom.pose.pose.position.y);
  p2.push_back(robot.plan[0].point.x);
  p2.push_back(robot.plan[0].point.y);

  if (p1[0]] != p2[0]] || p1[1]] != p2[1]])
  {
    double d = v*t/this->dist(p1,p2);
    if d <= 1.0
    {
      robot.odom.pose.pose.position.x += (p2[0]-p1[0])*d;
      robot.odom.pose.pose.position.y += (p2[1]-p1[1])*d;
    } else
    {
      robot.odom.pose.pose.position.x = p2[0];
      robot.odom.pose.pose.position.y = p2[1];
    }
  }
  

  if (robot.type == SCOUT)
  {
    switch (robot->code)
    {
    case ACTION_SCOUT_T::_planning:
      return; // no change in position or uncertainty
    case ACTION_SCOUT_T::_volatile_handler:
      // no clear way account change in position
      return; 
    case ACTION_SCOUT_T::_lost:
      return; 
    default:
       ROS_ERROR("Invalid robot code: time_to_motion");
  }

  if (robot.type == EXCAVATOR)
  {
    switch (robot->code)
    {
    case ACTION_HAULER_T::_planning:
      return; // no change in position or uncertainty
    case ACTION_HAULER_T::_volatile_handler:
      // no clear way account change in position
      return; 
    case ACTION_HAULER_T::_lost:
      return; 
    default: 
      ROS_ERROR("Invalid robot code: time_to_motion");

  }
  if (robot.type == HAULER)
  {
    switch (robot->code)
    {
    case ACTION_HAULER_T::_planning:
      return; // no change in position or uncertainty 
    case ACTION_HAULER_T::_volatile_handler:
      // no clear way account change in position
      
      return; 
    case ACTION_HAULER_T::_lost:
      
      return; 
    case ACTION_HAULER_T::_hauler_dumping:
      
      return; 
    default: 
      ROS_ERROR("Invalid robot code: time_to_motion");

  }
}

//------------------------------------------------------------------


int ForwardSearch::get_robot_index(int robot_type, int robot_id)
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

double ForwardSearch::dist(const std::vector<double> p1, const std::vector<double> p2) {
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

std::vector<std::vector<int>> cartesian_product(const std::vector<std::vector<int>> &v) {
  std::vector<std::vector<int>> s = {{}};
  for (const auto & u : v)
  {
    std::vector<std::vector<int>> r;
    for (const auto & x : s)
    {
      for (const auto & y : u)
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

  