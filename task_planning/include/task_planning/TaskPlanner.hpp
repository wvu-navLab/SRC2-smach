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


namespace mac {

class TaskPlanner {

  public:
    /** \brief  */
    TaskPlanner(const CostFunction       & cost_function,
                const std::vector<mac::Robot> & robots, const PlanningParams &planning_params);

    /** \brief  */
    void plan() const;

  protected:
    /** \brief  */
    PlanningParams planning_params_;

    /** \brief  */
    CostFunction cost_function_;

    /** \brief  */
    //std::vector<Volatile> volatiles_;

    /** \brief  */
    std::vector<mac::Robot> robots_;

    /** \brief  */
    ros::NodeHandle nh_;

    /** \brief  */
    ros::Subscriber sub_clock_, sub_volatiles_;

    /** \brief  */
    std::vector<ros::Subscriber> subs_robots_;

    /** \brief  */
    std::vector<ros::Publisher> pubs_plans_;

    /** \brief  */
    void timeCallback(const rosgraph_msgs::Clock::ConstPtr &msg);

    /** \brief  */
    //void volatileListCallback(const vol_data_type &msg);

    /** \brief  */
    void pose_callback(const ros::MessageEvent<std_msgs::Bool const>& event);
      //nav_msgs::Odometry
    /** \brief  */
    void monitor_callback(const ros::MessageEvent<std_msgs::Bool const>& event);


  private:
    const int SCOUT_STR_LOC = 13; //index ~SHOULD BE~ at 14th position
    const int EXCAVATOR_STR_LOC = 17; //index ~SHOULD BE~ at 18th position
    const int HAULER_STR_LOC = 14; //index ~SHOULD BE~ at 15th position

};

}

#endif // TaskPlanner_HPP
