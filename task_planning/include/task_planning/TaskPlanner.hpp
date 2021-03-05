// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file TaskPlanner.hpp
 */
//-----------------------------------------------------------------------------

#ifndef TaskPlanner_HPP
#define TaskPlanner_HPP

namespace mac {

class TaskPlanner {
  public:
    /** \brief  */
    inline TaskPlanner(const CostFunction       & cost_function,
                       const std::vector<Robot> & robots) {
      cost_function_ = cost_function;
      robots_ = robots;

      //setup robot subscribers
      int index_sub_scout = 1;
      int index_sub_excavator = 1;
      int index_sub_hauler = 1;
      for (int i=0; i<robots.size(); i++) {
        switch(robots[i].type) {
          case SCOUT:
            std::string topic = "/scout_" + std::to_string(index_sub_scout) + "/localization/odometry/sensor_fusion";
            subs_robots_[i] = nh_.subscribe(topic, 10, &TaskPlanner::scout_callback, this);
            index_sub_scout++;
            break;
          case EXCAVATOR:
            std::string topic = "/excavator_" + std::to_string(index_sub_excavator) + "/localization/odometry/sensor_fusion";
            subs_robots_[i] = nh_.subscribe(topic, 10, &TaskPlanner::excavator_callback, this);
            index_sub_excavator++;
            break;
          case HAULER:
            std::string topic = "/hauler_" + std::to_string(index_sub_hauler) + "/localization/odometry/sensor_fusion";
            subs_robots_[i] = nh_.subscribe(topic, 10, &TaskPlanner::hauler_callback, this);
            index_sub_hauler++;
            break;
          default:
            ROS_ERROR("TaskPlanner::TaskPlanner: robot type invalid!");
            break;
        }
      }

      //setup robot publishers
      int index_pub_scout = 1;
      int index_pub_excavator = 1;
      int index_pub_hauler = 1;
      for (int i=0; i<robots.size(); i++) {
        switch(robots[i].type) {
          case SCOUT:
            std::string topic = "/scout_" + std::to_string(index_pub_scout) + "/plan";
            pubs_plans_[i] = nh_.advertise<std_msgs::String>(topic, 10);
            index_pub_scout++;
            break;
          case EXCAVATOR:
            std::string topic = "/excavator_" + std::to_string(index_pub_excavator) + "/plan";
            pubs_plans_[i] = nh_.advertise<std_msgs::String>(topic, 10);
            index_pub_excavator++;
            break;
          case HAULER:
            std::string topic = "/hauler_" + std::to_string(index_pub_hauler) + "/plan";
            pubs_plans_[i] = nh_.advertise<std_msgs::String>(topic, 10);
            index_pub_hauler++;
            break;
          default:
            ROS_ERROR("TaskPlanner::TaskPlanner: robot type invalid!");
            break;
        }
      }

      //setup volatile subscribers
      //TODO
    }

    /** \brief  */
    void plan() const;

  protected:
    /** \brief  */
    CostFunction cost_function_;

    /** \brief  */
    std::vector<Volatile> volatiles_;

    /** \brief  */
    std::vector<Robot> robots_;

    /** \brief  */
    ros::NodeHandle nh_;

    /** \brief  */
    std::vector<ros::Subscriber> subs_volatiles_;

    /** \brief  */
    std::vector<ros::Subscriber> subs_robots_;

    /** \brief  */
    std::vector<ros::Publisher> pubs_plans_;

    /** \brief  */
    void scout_callback(const nav_msgs::Odometry::ConstPtr &msg);

    /** \brief  */
    void excavator_callback(const nav_msgs::Odometry::ConstPtr &msg);

    /** \brief  */
    void hauler_callback(const nav_msgs::Odometry::ConstPtr &msg);

};

}

#endif // TaskPlanner_HPP