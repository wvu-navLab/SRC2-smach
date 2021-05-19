/*
   Scouting Task Node
   scouting_task_node.cpp
   Purpose: Allocates scouts to perform search related tasks
   @author Jared Beard
   @version 1.0 2/19/21
 */
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>

#include <std_msgs/Bool.h>

#include <task_planning/Types.hpp>
#include <task_planning/TaskPlanner.hpp>
#include <task_planning/CostFunction.hpp>

void print_vector(std::vector<double> vec, std::string s) {
  std::cout << s << " = " << std::endl;
  for(auto & v:vec) {
    std::cout << v << ", ";
  }
  std::cout << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"scouting_task_node");
    ros::NodeHandle nh;

    /**----------------- Import Offline Plan -----------------------------*/

    /**----------------- Import Parameters -----------------------------*/
    int num_scouts, num_excavators, num_haulers;
    nh.getParam("/num_scouts",num_scouts);

    double max_time, timeout;
    nh.getParam("/clock_max_time_sec", max_time);

    std::vector<double> cf_params;
    nh.getParam("/cost_function/params", cf_params);
    std::string planner_type;
    int cost_type;
    nh.getParam("/cost_function/type", cost_type);
    nh.getParam("/planner/planning_type", planner_type);
    nh.getParam("/planner/planning_timeout_sec", timeout);

    int num_nodes, num_iterations;
    nh.getParam("/planner/num_nodes", num_nodes);
    nh.getParam("/planner/num_iterations", num_iterations);

    bool demo;
    nh.getParam("/demo", demo);

    std::vector<std::vector<double>> plan;
    std::vector<double> temp;
    nh.getParam("/waypoints/robot_type", temp);
    print_vector(temp,"robot_type");
    plan.push_back(temp);
    nh.getParam("/waypoints/robot_id", temp);
    print_vector(temp,"robot_id");
    plan.push_back(temp);
    nh.getParam("/waypoints/x", temp);
    print_vector(temp,"x");
    plan.push_back(temp);
    nh.getParam("/waypoints/y", temp);
    print_vector(temp,"y");
    plan.push_back(temp);

    // ROS_INFO_STREAM("Plan: " << plan);
    std::cout << "Plan 1:" << std::endl;
    for(auto & ps:plan) {
      std::cout << std::endl;
      for(auto & p:ps) {
        std::cout << "p = " << p << ",";
      }
    }
    std::cout << std::endl;
    
    /**----------------- Initialize -----------------------------*/
    // Initialize Robots
    std::vector<mac::Robot> robots;
    mac::Robot rbt;
    for (int i = 0; i < num_scouts; ++i)
    {
      rbt.id = i+1;
      rbt.type = mac::SCOUT;
      robots.push_back(rbt);
    }

    // Initialize Cost Function
    const mac::CostFunction cf(cost_type);

    mac::PlanningParams planning_params;
    planning_params.max_time = max_time;
    planning_params.timeout = timeout;
    planning_params.demo = demo;
    planning_params.type = mac::SCOUT_PLANNER_DEFAULT;
    planning_params.plan = plan;
    // ROS_INFO_STREAM("Params Plan: " << planning_params.plan);
    std::cout << "Plan 2:" << std::endl;
    for(auto & ps:planning_params.plan) {
      std::cout << std::endl;
      for(auto & p:ps) {
        std::cout << "p = " << p << ",";
      }
    }
    std::cout << std::endl;


    std::cout << "here1" << std::endl;
    mac::TaskPlanner tp(cf,robots, planning_params);


    // ros::Publisher pub = nh.advertise<std_msgs::Bool>("/small_excavator_1/localization/odometry/sensor_fusion", 10);
    std::cout << "here2" << std::endl;
    std_msgs::Bool msg;
    /**----------------- Initialize Rover Metric Class------------------------------*/
    ros::spin();

    return 0;
}
