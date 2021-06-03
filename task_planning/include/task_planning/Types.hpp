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

namespace mac
{

enum class ACTION_HAULER_T {_initialize=0, _planning=1, _traverse=2, _volatile_handler=3, _lost=4, _hauler_dumping=5, _in_progress=6};
enum class ACTION_EXCAVATOR_T {_initialize=0, _planning=1, _traverse=2, _volatile_handler=3, _lost=4, _in_progress=5};
enum class ACTION_SCOUT_T {_initialize=0, _planning=1, _traverse=2, _volatile_handler=3, _lost=4, _in_progress=5};

const int SCOUT = 0;
const int EXCAVATOR = 1;
const int HAULER = 2;

struct Robot {
    int id;
    int type;
    int volatile_index;
    double time_remaining; //0 = not start, 1 = full, -1 = failed
    double current_task = -1;
    double bucket_contents; //0 = empty, 1 = full
    nav_msgs::Odometry odom;
    std::vector<geometry_msgs::PointStamped> plan;
    bool is_initialized = false;
};

struct Volatile {
    std::string id;
    int points;
    bool is_initialized = false;
};

struct State
{
  std::vector<Robot> robots;
  volatile_map::VolatileMap volatile_map;
  ros::Time time;
};

struct Action
{
  std::pair<double, double> objective;
  int robot_type;
  int id;
  int code;
  int volatile_index;
  bool toggle_sleep;
};

struct PlanningParams {
    int max_time; // seconds
    int timeout;
    bool demo;
    int type;
    std::vector<std::vector<double>> plan;
    // Environment 
    double wait_time; //seconds
    // list parameters here such as power depletion rates, velocity, etc

};



}

#endif // Types_HPP
