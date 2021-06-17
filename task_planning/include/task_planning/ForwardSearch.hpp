// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file ForwardSearch.hpp
 */
//-----------------------------------------------------------------------------

#ifndef ForwardSearch_HPP
#define ForwardSearch_HPP

#include <string>
#include <vector>
#include <numeric>

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

namespace mac
{

  class ForwardSearch
  {
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
    /** \brief  */
    ForwardSearch();

    /** \brief  */
    ForwardSearch(const CostFunction &cost_function,
                  const PlanningParams &planning_params);

    /** \brief  Run the forward search, which considers all possible actions
   * sequences given a specified horizon (i.e., maximum depth of the tree)
  */
    Action plan(const State &s);

    /** \brief  */
    bool reinit();

  protected:
    /** \brief  Tree vertex used in the forward search */
    struct Vertex
    {
      State state;
      double cost;

      int parent_layer_index;
      int depth;
      int layer_index;
      std::vector<int> children;
    };

    /** \brief  */
    std::vector<vertex> expand(const State &s,
                               const std::vector<Action> &joint_action);

    /** \brief Return all possible actions for the system given the state where
   * the actions are the combinations of possible actions for the robots.*/
    std::vector<std::vector<Action>> get_actions_all_robots(const State &s);

    /** \brief  Return all possible actions for a robot given the state. */
    std::vector<Action> get_actions_robot(int robot_index, const State &s);

    /** \brief  Return all possible actions for a scout given the state */
    std::vector<Action> ForwardSearch::get_actions_scout(int robot_index, const State &s);

    /** \brief  Return all possible actions for an excavator given the state */
    std::vector<Action> ForwardSearch::get_actions_excavator(int robot_index, const State &s);

    /** \brief  Return all possible actions for a hauler given the state */
    std::vector<Action> ForwardSearch::get_actions_hauler(int robot_index, const State &s);

    //INCOMPLETE
    /** \brief  */
    int get_policy();

    /** \brief  */
    vertex propagate(State s, Action a);

    //INCOMPLETE
    /** \brief  */
    void get_state(State s);

    /** \brief  */
    int get_parent(int ind_v, int depth);

    /** \brief  */
    CostFunction cost_function_;

    /** \brief  */
    PlanningParams planning_params_;

    //TODO(Jared): do we want to represent the tree as a vector instead of a vector of vectors
    //             to be consistent with the toolbox?
    /** \brief  */
    std::vector<std::vector<Vertex>> tree_;

  private:
    double simulate_time_remaining(Robot *robot);
    double get_traverse_time(Robot &robot);
    double get_handling_time(Robot &robot);
    double get_homing_time(Robot &robot);
    double get_dumping_time(Robot &robot);

    std::vector<double> actions_to_time(const State &s,
                                        const std::vector<Action> &joint_action);
    double action_to_time(const State &s,
                          const Action &a);
    double get_traverse_time(Robot &robot);
    double get_handling_time(Robot &robot);
    double get_homing_time(Robot &robot);
    double get_dumping_time(Robot &robot);
    void simulate_time_step(Robot &robot,
                            double time);
    double time_to_power(Robot &robot,
                         double time);
    double time_to_motion(Robot &robot,
                          double time);

    /** \brief  */
    std::vector<std::vector<int>> cartesian_product(const std::vector<std::vector<int>> &v);

    int get_robot_index(int robot_type, int robot_id);
  };

}

#endif
