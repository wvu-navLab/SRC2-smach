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

#include <task_planning/TaskPlanner.hpp>
#include <task_planning/CostFunction.hpp>

int main(int argc, char** argv)
{
		ros::init(argc,argv,"scouting_task_node");
		ros::NodeHandle nh;

    /**----------------- Import Offline Plan -----------------------------*/

    /**----------------- Import Parameters -----------------------------*/

    /**----------------- Initialize Pubs/Subs -----------------------------*/
		//ros::Subscriber sub = nh.subscribe("/velocities", 10, motorCommandCallback);
		//ros::Publisher pub = n.advertise<std_msgs::Int64MultiArray>("/motor_pos", 1000);
    //ros::Publisher pubCurrent = n.advertise<std_msgs::Int64MultiArray>("/motor_currents", 1000);
		const mac::CostFunction cf("type1");
		mac::TaskPlanner tp(cf);

		ros::Publisher pub = nh.advertise<std_msgs::Bool>("/small_scout_1/localization/odometry/sensor_fusion", 10);
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
