/*
   Excavation Task Node
   excavation_task_node.cpp
   Purpose: Allocates excavators and haulers to perform excavations related tasks
   @author Jared Beard
   @version 1.0 2/19/21
 */
#include <ros/ros.h>
#include <ros/console.h>

int main(int argc, char** argv)
{
		ros::init(argc,argv,"scouting_task_node");
		ros::NodeHandle nh;

    /**----------------- Import Parameters -----------------------------*/

    /**----------------- Initialize Pubs/Subs -----------------------------*/
		//ros::Subscriber sub = nh.subscribe("/velocities", 10, motorCommandCallback);
		//ros::Publisher pub = n.advertise<std_msgs::Int64MultiArray>("/motor_pos", 1000);
    //ros::Publisher pubCurrent = n.advertise<std_msgs::Int64MultiArray>("/motor_currents", 1000);

    /**----------------- Initialize Volatile Metric Class------------------------------*/

    /**----------------- Initialize Rover Metric Class------------------------------*/

    while(ros::ok())
    {
      /**----------------- Update Volatile Cost ------------------------------*/

      /**----------------- Update Excavator Cost ------------------------------*/

      /**----------------- Update Hauler Cost ------------------------------*/

      /**----------------- Perform Online Planning ------------------------------*/
      // This could be a part of the cost steps.

      /**----------------- Publish Objectives ------------------------------*/
    }

    return 0;
}
