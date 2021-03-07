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
#include <ros/console.h>
#include <ros/transport_hints.h>

#include <task_planning/CostFunction.hpp>


namespace mac {

class TaskPlanner {
  public:
    /** \brief  */
    TaskPlanner(const CostFunction       & cost_function);//,
                       //const std::vector<Robot> & robots);

    /** \brief  */
    void plan() const;

  protected:
    /** \brief  */
    CostFunction cost_function_;

    /** \brief  */
    //std::vector<Volatile> volatiles_;

    /** \brief  */
    //std::vector<Robot> robots_;

    /** \brief  */
    ros::NodeHandle nh_;

    /** \brief  */
    std::vector<ros::Subscriber> subs_volatiles_;

    /** \brief  */
    //std::vector<
    ros::Subscriber subs_robots_;

    /** \brief  */
    std::vector<ros::Publisher> pubs_plans_;

    /** \brief  */
    void timeCallback(const rosgraph_msgs::Clock::ConstPtr &msg);

    /** \brief  */
    //void volatileListCallback(const vol_data_type &msg);

    /** \brief  */
    void scout_pose_callback(const ros::MessageEvent<std_msgs::Bool const>& event);

    /** \brief  */
    void scout_monitor_callback(const ros::MessageEvent<std_msgs::String const>& event);

    /** \brief  */
    void excavator_pose_callback(const ros::MessageEvent<std_msgs::String const>& event);

    /** \brief  */
    void excavator_monitor_callback(const ros::MessageEvent<std_msgs::String const>& event);

    /** \brief  */
    void hauler_pose_callback(const ros::MessageEvent<std_msgs::String const>& event);

    /** \brief  */
    void hauler_monitor_callback(const ros::MessageEvent<std_msgs::String const>& event);

    long unsigned int SCOUT_STR_LOC = 13; //index ~SHOULD BE~ at 14th position
    long unsigned int EXCAVATOR_STR_LOC = 17; //index ~SHOULD BE~ at 18th position
    long unsigned int HAULER_STR_LOC = 14; //index ~SHOULD BE~ at 15th position

};

}

#endif // TaskPlanner_HPP
