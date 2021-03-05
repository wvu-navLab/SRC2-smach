// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file Types.hpp
 */
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

#ifndef Types_HPP
#define Types_HPP

namespace mac
{

const SCOUT = 0;
const EXCAVATOR = 1;
const HAULER = 2;

struct Robot {
    std::string id;
    std::string type;
    nav_msgs::Odometry odom;
    bool is_initialized = false;
};

struct Volatile {
    std::string id;
    int points;
    bool is_initialized = false;
};

}

#endif // Types_HPP