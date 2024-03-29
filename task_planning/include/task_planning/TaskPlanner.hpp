// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file TaskPlanner.hpp
 */
//-----------------------------------------------------------------------------

#ifndef TaskPlanner_HPP
#define TaskPlanner_HPP

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
#include <task_planning/DumpCoordination.h>
#include <task_planning/ForwardSearch.hpp>
#include <volatile_map/VolatileMap.h>
#include <geometry_msgs/PointStamped.h>





//TODO:
//X  Make a diagram for interaction between high level planner and state machines
//X  Test on some dummy variables
//X  figure out volatile representation -> tag-up with Bernardo/Jason/Cagri to settle on interface
//  demo planner/cost functions
//X  add variables for time/pose/plan/etc
//X  settle on when to trigger planning (currently think robots do it as needed, alternative is to trigger based on event updates such as new volatile added to list)
//  add in interrupt functionality so that high level planner can override state machine's objectives
//      need to make sure that if robot is homing that it doesn't necessarily cancel that mission
//  adapt nodes to operate as services

// NOTE: plan will just give robot its current objective, not all objectives

// Planner maintain queue of plans
// Robot calls planner and whether it was successful or as state of the robot

//WARNING:
//planner sets volatiles forever

//TODO:
//maybe, change the publisher to a service that can be called by robots
//need a status topic from each robot, so the planner can know what the robots are doing
//need to add proper planner
//



namespace mac {

const int SCOUT_PLANNER_DEFAULT = 0;
const int EXC_HAUL_PLANNER_DEFAULT = 1;
const int EXC_HAUL_PLANNER_DEFAULT_ADV = 1;
const int EXC_HAUL_FORWARD_SEARCH = 2;

class TaskPlanner {

  public:

    /** \brief  */
    TaskPlanner(const CostFunction            & cost_function,
                const std::vector<mac::Robot> & robots,
                const PlanningParams          & planning_params);

  protected:
    /** \brief  */
    PlanningParams planning_params_;

    /** \brief  */
    CostFunction cost_function_;

    /** \brief  */
    volatile_map::VolatileMap volatile_map_;

    /** \brief  */
    std::vector<mac::Robot> robots_;

    /** \brief  */
    ros::NodeHandle nh_;

    /** \brief  */
    ros::Subscriber sub_clock_, sub_volatiles_;

    /** \brief */
    ros::Publisher pub_interrupt;

    /** \brief  */
    std::vector<ros::Subscriber> subs_robots_;

    /** \brief  */
    std::vector<ros::Publisher> pubs_plans_;

    /** \brief  */
    ros::Time time_;

    /** brief */
    ros::ServiceServer server_task_planner;

    ros::ServiceServer server_dump_request;

    /** \brief  */
    ForwardSearch forward_search_;

    /** \brief  */
    void timeCallback(const rosgraph_msgs::Clock::ConstPtr &msg);

    /** \brief  */
    void volatileMapCallback(const volatile_map::VolatileMap::ConstPtr &msg);

    /** \brief  */
    void poseCallback(const ros::MessageEvent<nav_msgs::Odometry const>& event);
      //nav_msgs::Odometry
    /** \brief  */
    //void taskStatusCallback(const ros::MessageEvent<std_msgs::Bool const>& event);
    /** \brief */
    bool taskPlanService(task_planning::PlanInfo::Request &req, task_planning::PlanInfo::Response &res);


    bool dumpRequestService(task_planning::DumpCoordination::Request &req, task_planning::DumpCoordination::Response &res);

    //maybe track internal state of robots or something along the lines of that
      // how full is hauler
      // was task successful
      //etc

    //may need a callback for prior map (eg coverage in search planning)


  private:
    bool hauler_dumping = false;
    const int SCOUT_STR_LOC = 13; //index ~SHOULD BE~ at 14th position
    const int EXCAVATOR_STR_LOC = 17; //index ~SHOULD BE~ at 18th position
    const int HAULER_STR_LOC = 14; //index ~SHOULD BE~ at 15th position

    int get_robot_index(int robot_type, int robot_id);
    static double dist(const std::vector<double> p1, const std::vector<double> p2);

    /** \brief  */
    void scout_plan_default(int type, int id);
    /** \brief  */
    void exc_haul_plan_default();
    void exc_haul_plan_default_adv();
    void closest_volatile_to_rover(volatile_map::VolatileMap &temp_map, std::vector<int> &temp_volatile_indices);

    void exc_haul_plan_fs(mac::State s);
    /** \brief */
    void populate_prior_plan();

    int plan_call_counter = 0;
};

}

#endif // TaskPlanner_HPP
