// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file ForwardSearch.hpp
 */
//-----------------------------------------------------------------------------

#ifndef ForwardSearch_HPP
#define ForwardSearch_HPP

#include <string>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/transport_hints.h>


#include <task_planning/Types.hpp>
#include <task_planning/CostFunction.hpp>
// #include <state_machine/RobotStatus.h>
#include <task_planning/PlanInfo.h>
#include <volatile_map/VolatileMap.h>
#include <geometry_msgs/PointStamped.h>



namespace mac {

struct vertex
{
  std::vector<Robot> rbts;
  volatile_map::VolatileMap volmap;
  ros::Time t;
  double cost;

  int parent;
  int depth;
  int index;
  std::vector<int> children;

};

class ForwardSearch {
    /**
    // settings
        If time of completion within epsilon, wrap them into one state transition?


    //  Environment:


    //  States:
          Robots:
            x,y,z
            P
            power
            time remaining for task
          Volatiles:
            x,y,z
            P
          Competition time/time remaining

    // Actions:

          Robots:
            Continue current task -> make default if a robot is doing something
            Toggle Sleep mode -- (Incurs speed penalty)
            Go to
             & do nothing/wait
             & Lost state
             & Handle Volatile
             & Dump (Haulers)

    // Transition:
          Robots:
            power depletion rate
            position Transition
            Uncertainty growth

          Time:
            map completion -> time remaining

          Power:
            Map speed/output to power level
            Map charging/depletion speed to direction
            Map depletion to motion
            Map excavation to power rate
            Immediate charging at base station

    */

public:
  ForwardSearch();
  ForwardSearch(const CostFunction  cost_function,
                const PlanningParams planning_params);

  int plan(std::vector<Robot> &robots,
            const volatile_map::VolatileMap &volatile_map,
            const ros::Time &time);
  bool reinit();

protected:
  std::vector<vertex> expand(vertex v, int actions);
  std::vector<int> get_actions(vertex v, std::vector<int> a);
  int get_policy();
  vertex propagate(vertex v, int action);
  void get_state(std::vector<Robot> robots,
                 volatile_map::VolatileMap volatile_map,
                 ros::Time time);
  int get_parent(int ind_v,int depth);

  CostFunction cost_function_;

  PlanningParams planning_params_;

  std::vector<Robot> robots_;

  volatile_map::VolatileMap volatile_map_;

  ros::Time time_;

  std::vector<std::vector<vertex>> tree;
private:

};

}

#endif
