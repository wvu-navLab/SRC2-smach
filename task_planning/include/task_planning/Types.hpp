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

namespace mac
{

const int SCOUT = 0;
const int EXCAVATOR = 1;
const int HAULER = 2;

struct Robot {
    int id;
    int type;
    double status;
    double current_task;
    nav_msgs::Odometry odom;
    bool is_initialized = false;
};

struct Volatile {
    std::string id;
    int points;
    bool is_initialized = false;
};

struct PlanningParams {
    int max_time;
    int timeout;
    bool demo;
};

}

#endif // Types_HPP
