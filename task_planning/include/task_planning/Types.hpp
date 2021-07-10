// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file Types.hpp
 */
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

#ifndef Types_HPP
#define Types_HPP

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <volatile_map/VolatileMap.h>

namespace mac
{

  enum class ACTION_HAULER_T
  {
    _initialize = 0,
    _planning = 1,
    _traverse = 2,
    _volatile_handler = 3,
    _lost = 4,
    _hauler_dumping = 5,
    _in_progress = 6
  };

  const char *ACTION_HAULER_T_STRINGS[] =
      {
          "_initialize",
          "_planning",
          "_traverse",
          "_volatile_handler",
          "_lost",
          "_hauler_dumping",
          "_in_progress"};

  enum class ACTION_EXCAVATOR_T
  {
    _initialize = 0,
    _planning = 1,
    _traverse = 2,
    _volatile_handler = 3,
    _lost = 4,
    _in_progress = 5
  };

  const char *ACTION_EXCAVATOR_T_STRINGS[] =
      {
          "_initialize",
          "_planning",
          "_traverse",
          "_volatile_handler",
          "_lost",
          "_in_progress"};

  enum class ACTION_SCOUT_T
  {
    _initialize = 0,
    _planning = 1,
    _traverse = 2,
    _volatile_handler = 3,
    _lost = 4,
    _in_progress = 5
  };

  const char *ACTION_SCOUT_T_STRINGS[] =
      {
          "_initialize",
          "_planning",
          "_traverse",
          "_volatile_handler",
          "_lost",
          "_in_progress"};

  const int SCOUT = 0;
  const int EXCAVATOR = 1;
  const int HAULER = 2;

  struct Robot
  {
    int id;
    int type;
    int volatile_index;
    double time_remaining; //0 = not start, 1 = full, -1 = failed
    int current_task = -1;
    double bucket_contents; //0 = empty, 1 = full
    nav_msgs::Odometry odom;
    std::vector<geometry_msgs::PointStamped> plan;
    bool is_initialized = false;
    bool toggle_sleep = false;
    double power;
  };

  struct State
  {
    std::vector<Robot> robots;
    volatile_map::VolatileMap volatile_map;
    ros::Time time;
    double time_elapsed = 0;
  };

  struct Action
  {
    int robot_type;
    std::pair<double, double> objective;
    int id;
    int code;
    int volatile_index;
    bool toggle_sleep;
    double action_time;
  };

  struct PlanningParams
  {
    int max_time;  // seconds
    int max_depth; //max depth to construct tree
    int timeout;
    bool demo;
    int type;
    std::vector<std::vector<double>> plan;
    // Environment
    double wait_time = 30;  //seconds
    double max_v_scout;     // m/s
    double max_v_excavator; // m/s
    double max_v_hauler;    // m/s
    double vol_handling_time_scout;
    double vol_handling_time_excavator;
    double vol_handling_time_hauler;
    double homing_time_scout;
    double homing_time_excavator;
    double homing_time_hauler;
    double dumping_time;
    std::vector<double> planning_scout_power_weights;
    std::vector<double> vol_handle_scout_power_weights;
    std::vector<double> lost_scout_power_weights;
    std::vector<double> planning_excavator_power_weights;
    std::vector<double> vol_handle_excavator_power_weights;
    std::vector<double> lost_excavator_power_weights;
    std::vector<double> planning_hauler_power_weights;
    std::vector<double> vol_handle_hauler_power_weights;
    std::vector<double> lost_hauler_power_weights;

    //  std::vector<double> lost_hauler_power_weights;

    std::vector<double> power_rates;

    // list parameters here such as power depletion rates, velocity, etc
  };

}

#endif // Types_HPP
