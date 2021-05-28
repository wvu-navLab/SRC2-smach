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

struct Vertex
{
  State s;
  double cost;

  int parent;
  int depth;
  int index;
  std::vector<int> children;

};

struct State {
  std::vector<Robot> robots;
  volatile_map::VolatileMap volatile_map;
  ros::Time time;
};

struct Action {
  std::pair<double, double> objective
  int robot_type;
  int id;
  int code;
  int volatile_index;
  bool toggle_sleep;
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

  Action plan(State s);
  bool reinit();

protected:
  std::vector<vertex> expand(State s, std::vector<Action> actions);
  std::vector<Action> get_actions_all(State s);
  std::vector<Action> get_actions_robot(int robot_index, State s);

  int get_policy();
  vertex propagate(State s, Action a);
  void get_state(State s);
  int get_parent(int ind_v,int depth);

  CostFunction cost_function_;

  PlanningParams planning_params_;

  State state_;

  std::vector<std::vector<Vertex>> tree;
private:
  double simulate_time_remaining(Robot *robot);
  void simulate_time_step(Robot *robot, double min_time)


  int get_robot_index(int robot_type, int robot_id);

};

}

#endif
