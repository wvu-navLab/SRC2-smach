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

int main(int argc, char** argv)
{
		ros::init(argc,argv,"scouting_task_node");
		ros::NodeHandle nh;

    /**----------------- Import Offline Plan -----------------------------*/

    /**----------------- Import Parameters -----------------------------*/
		int num_excavators, num_haulers;
		nh.getParam("/num_excavators",num_excavators);
		nh.getParam("/num_haulers",num_haulers);


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

    /**----------------- Initialize -----------------------------*/
		// Initialize Robots
		std::vector<mac::Robot> robots;
		mac::Robot rbt;
		for (int i = 0; i < num_excavators; ++i)
		{
			rbt.id = i+1;
			rbt.type = mac::EXCAVATOR;
			robots.push_back(rbt);
		}
		for (int i = 0; i < num_haulers; ++i)
		{
			rbt.id = i+1;
			rbt.type = mac::HAULER;
			robots.push_back(rbt);
		}
		// Initialize Cost Function
		const mac::CostFunction cf(cost_type);

		mac::PlanningParams planning_params;
		planning_params.max_time = max_time;

		mac::TaskPlanner tp(cf,robots, planning_params);

		ros::Publisher pub = nh.advertise<std_msgs::Bool>("/small_excavator_1/localization/odometry/sensor_fusion", 10);
		std_msgs::Bool msg;
    /**----------------- Initialize Rover Metric Class------------------------------*/
		ros::Rate rate(10);
    while(ros::ok())
    {

      /**----------------- Update Rover Cost ------------------------------*/

      /**----------------- Perform Online Planning ------------------------------*/
      // This could be a part of the cost steps.

      /**----------------- Publish Objectives ------------------------------*/
			msg.data = true;
			pub.publish(msg);

			ros::spinOnce();
    }

    return 0;
}
