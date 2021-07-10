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
#include <random>
#include <algorithm>

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

//TODO: add is_terminal() behavior

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

   const bool disable_planning_action_ = true;
   const bool disable_lost_action_ = true;

    std::mt19937_64 rng_;
    std::uniform_real_distribution<double> unif_;

  public:
    /** \brief  */
    //ForwardSearch();

    /** \brief  */
    ForwardSearch(const CostFunction cost_function,
                  const PlanningParams planning_params);

    /** \brief  Run the forward search, which considers all possible actions
     * sequences given a specified horizon (i.e., maximum depth of the tree)
     */
    std::vector<Action> plan(const State &s);

    //debugging
    std::vector<std::vector<Action>> get_sequence_of_joint_actions(int depth, int layer_index);
    std::vector<std::vector<std::vector<Action>>> get_all_sequences_of_joint_actions();
    std::vector<std::vector<Action>> get_best_sequence_of_joint_actions();

    /** \brief  */
    //TODO: move to private after moving test propagate funtion to ForwardSearch
    State propagate(const State &s,
                    const std::vector<Action> &joint_action);

    /** \brief Return all possible actions for the system given the state where
     * the actions are the combinations of possible actions for the robots.
     */
    std::vector<std::vector<Action>> get_actions_all_robots(const State &s);

    /** \brief  */
    // bool reinit();

    //print commands
    void print_joint_action(std::vector<Action> joint_action);
    void print_sequence_of_joint_actions(std::vector<std::vector<Action>> joint_actions);
    void print_volatile_map(volatile_map::VolatileMap volatile_map);
    void print_robot(int robot_index, std::vector<Robot> robots);
    void print_robots(std::vector<Robot> robots);
    void print_state(State s);

  protected:

    /** \brief  Tree vertex used in the forward search */
    struct Vertex
    {
      State state;
      std::vector<Action> joint_action; //action to get to this vertex
      double cost;
      double total_cost;

      int parent_layer_index;
      int depth;
      int layer_index;
      std::vector<int> children;
    };

    /** \brief  Return all possible actions for a robot given the state. */
    std::vector<Action> get_actions_robot(int robot_index, const State &s);

    /** \brief  Return all possible actions for a scout given the state */
    std::vector<Action> get_actions_scout(int robot_index, const State &s);

    /** \brief  Return all possible actions for an excavator given the state */
    std::vector<Action> get_actions_excavator(int robot_index, const State &s);

    /** \brief  Return all possible actions for a hauler given the state */
    std::vector<Action> get_actions_hauler(int robot_index, const State &s);

    /** \brief  */
    std::vector<Action> get_policy();

    /** \brief  */
    // void get_state(State s);

    /** \brief  */
    const CostFunction cost_function_;

    /** \brief  */
    const PlanningParams planning_params_;

    //TODO(Jared): do we want to represent the tree as a vector instead of a vector of vectors
    //             to be consistent with the toolbox?
    /** \brief  */
    std::vector<std::vector<Vertex>> tree_;

  private:
    /** \brief  */
    std::vector<double> actions_to_time(const State &s,
                                        const std::vector<Action> &joint_action);
    /** \brief  */
    double action_to_time(const State &s,
                          const Action &a);

    /** \brief  */
    double get_traverse_time(Robot &robot);

    /** \brief  */
    double get_handling_time(Robot &robot);

    /** \brief  */
    double get_homing_time(Robot &robot);

    /** \brief  */
    double get_dumping_time(Robot &robot);

    /** \brief  */
    void simulate_time_step(Robot &robot,
                            double time);

    /** \brief  */
    //double time_to_power(Robot &robot,
    //                     double time);

    /** \brief  */
    double time_to_motion(Robot &robot,
                          double time);

    /** \brief  */
    std::vector<std::vector<int>> cartesian_product(const std::vector<std::vector<int>> &v);

    /** \brief  */
    int get_robot_index(const State &s, int robot_type, int robot_id);

    /** \brief  */
    double dist(const std::vector<double> p1,
                const std::vector<double> p2);
  };

}

#endif
