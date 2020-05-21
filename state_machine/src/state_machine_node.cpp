/**
@state_machine_node state_machine_node.cpp
Performs low level state machine action items for SRC2.
*/

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

#include <srcp2_msgs/ResetModelSrv.h>
#include <srcp2_msgs/BrakeRoverSrv.h>

#include <motion_control/JointGroup.h>
#include <driving_tools/Stop.h>

#include <iostream>

//flags for when systems  receive update or are given a go ahead
bool PLANNER_FLAG = false, LOCALIZATION_FLAG = false, ALIGNMENT_FLAG = false, EXCAVATOR_FLAG = false, RESET_FLAG = false, SYSTEM_FAILURE_FLAG = false, ARM_FLAG = false;

geometry_msgs::PoseStamped PREVIOUS_GOAL, PLANNER_GOAL, LOCALIZATION_GOAL, EXCAVATOR_GOAL, ALIGNMENT_GOAL;

motion_control::JointGroup ARM_GOAL;

bool poseStampedCompare(geometry_msgs::PoseStamped p1, geometry_msgs::PoseStamped p2)
{

  bool check_1 = p1.header.stamp.toSec() == p2.header.stamp.toSec();
  bool check_2 = p1.pose.position.x == p2.pose.position.x;
  bool check_3 = p1.pose.position.y == p2.pose.position.y;
  bool check_4 = p1.pose.position.z == p2.pose.position.z;
  bool check_5 = p1.pose.orientation.x == p2.pose.orientation.x;
  bool check_6 = p1.pose.orientation.y == p2.pose.orientation.y;
  bool check_7 = p1.pose.orientation.z == p2.pose.orientation.z;
  bool check_8 = p1.pose.orientation.w == p2.pose.orientation.w;

  return check_1 && check_2 && check_3 && check_4 && check_5 && check_6 && check_7 && check_8;
}

void arm_callback(const motion_control::JointGroupConstPtr &msg)
{
  ARM_GOAL = *msg;
}

void arm_on_callback(const std_msgs::BoolConstPtr &msg)
{
  ARM_FLAG = msg->data;
}

void planner_callback(const geometry_msgs::PoseStampedConstPtr &msg)
{

  if(msg->header.stamp.toSec() == -1)
  {

    PLANNER_FLAG = false;
  } else if (!poseStampedCompare(*msg,PREVIOUS_GOAL))
  {

    PLANNER_GOAL = *msg;
    PLANNER_FLAG = true;
  } else
  {

  }
}

void localization_callback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  if(msg->header.stamp.toSec() == -1)
  {
    LOCALIZATION_FLAG = false;
  } else if (!poseStampedCompare(*msg,PREVIOUS_GOAL))
  {
    LOCALIZATION_GOAL = *msg;
    LOCALIZATION_FLAG = true;
  }
}

/**void excavation_callback()
{

}*/

void alignment_callback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  if(msg->header.stamp.toSec() == -1)
  {
    ALIGNMENT_FLAG = false;
  } else if (!poseStampedCompare(*msg,PREVIOUS_GOAL))
  {
    ALIGNMENT_GOAL = *msg;
    ALIGNMENT_FLAG = true;
  }
}

void reset_callback(const std_msgs::BoolConstPtr &msg)
{
  RESET_FLAG = msg->data;
}

void failure_callback(const std_msgs::BoolConstPtr &msg)
{
  SYSTEM_FAILURE_FLAG = msg->data;
}

int main(int argc, char **argv)
{
  // Initialize ROS, Subs, and Pubs *******************************************
  ros::init(argc, argv, "state_machine_node");

  ros::NodeHandle nh;

  double wait_time;
  nh.getParam("/reset_model_delay",wait_time);

  ros::Subscriber planner_sub = nh.subscribe("planner_goal",1,planner_callback);
  ros::Subscriber localization_sub = nh.subscribe("localization_goal",1,localization_callback);
  ros::Subscriber alignment_sub = nh.subscribe("alignment_goal",1,alignment_callback);
  //ros::Subscriber excavation_sub = nh.Subscribe(excavation_goal,1,excavation_callback);
  ros::Subscriber reset_sub = nh.subscribe("hard_reset",1,reset_callback);
  ros::Subscriber failure_sub = nh.subscribe("system_failure",1,failure_callback);
  ros::Subscriber arm_sub = nh.subscribe("arm_goal",1,arm_callback);
  ros::Subscriber arm_on_sub = nh.subscribe("arm_on",1,arm_on_callback);


  ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal",1);
  ros::Publisher arm_goal_pub = nh.advertise<motion_control::JointGroup>("jointAngles",1);
  //ros::Publisher arm_goal_pub = nh.advertise<>();

  ros::ServiceClient reset_client = nh.serviceClient<srcp2_msgs::ResetModelSrv>("reset_model");
  ros::ServiceClient brake_client = nh.serviceClient<srcp2_msgs::BrakeRoverSrv>("brake_rover");

  ros::ServiceClient arm_brake_client = nh.serviceClient<driving_tools::Stop>("stop");


  // Start loop and initialize other values*************************************
  bool goal_update = false;
  bool brake_flag = false;
  srcp2_msgs::ResetModelSrv reset_srv;
  srcp2_msgs::BrakeRoverSrv brake_srv;
  brake_srv.request.brake = true;
  brake_client.call(brake_srv);

  driving_tools::Stop arm_brake_srv;
  arm_brake_srv.request.enableStop = true;

  ros::Rate rate(.5);

  ros::Time temp_dur;
  temp_dur.sec = -2;
  PREVIOUS_GOAL.header.stamp = temp_dur;
  PREVIOUS_GOAL.pose.position.x = 0;
  PREVIOUS_GOAL.pose.position.y = 0;
  PREVIOUS_GOAL.pose.position.z = 0;
  PREVIOUS_GOAL.pose.orientation.x = 0;
  PREVIOUS_GOAL.pose.orientation.y = 0;
  PREVIOUS_GOAL.pose.orientation.z = 0;
  PREVIOUS_GOAL.pose.orientation.w = 0;

  ROS_WARN("%f",wait_time);

  while(ros::ok())
  {
  geometry_msgs::PoseStamped overall_goal;

  if (SYSTEM_FAILURE_FLAG)
  {// SYSTEM FAILURE CONDITION - wait ******************************************
    brake_srv.request.brake = true;
    brake_client.call(brake_srv);
    ROS_WARN("SM: SYSTEM_FAILURE");
    brake_flag = true;
    PREVIOUS_GOAL.header.stamp = temp_dur;
    PREVIOUS_GOAL.pose.position.x = 0;
    PREVIOUS_GOAL.pose.position.y = 0;
    PREVIOUS_GOAL.pose.position.z = 0;
    PREVIOUS_GOAL.pose.orientation.x = 0;
    PREVIOUS_GOAL.pose.orientation.y = 0;
    PREVIOUS_GOAL.pose.orientation.z = 0;
    PREVIOUS_GOAL.pose.orientation.w = 0;
    arm_brake_client.call(brake_srv);
  } else if(RESET_FLAG)
  {// RESET ********************************************************************
      reset_srv.request.reset = true;
      if (reset_client.call(reset_srv))
      {
        brake_srv.request.brake = true;
        brake_client.call(brake_srv);
        ROS_WARN("SM: SYSTEM_FAILURE");
        brake_flag = true;
        PREVIOUS_GOAL.header.stamp = temp_dur;
        PREVIOUS_GOAL.pose.position.x = 0;
        PREVIOUS_GOAL.pose.position.y = 0;
        PREVIOUS_GOAL.pose.position.z = 0;
        PREVIOUS_GOAL.pose.orientation.x = 0;
        PREVIOUS_GOAL.pose.orientation.y = 0;
        PREVIOUS_GOAL.pose.orientation.z = 0;
        PREVIOUS_GOAL.pose.orientation.w = 0;
        ros::Duration(wait_time).sleep();
        ROS_WARN("SM: ROBOT RESET");
      }
      RESET_FLAG = false;
      arm_brake_client.call(brake_srv);
  } else if(LOCALIZATION_FLAG)
  {// LOST *********************************************************************
    goal_update = true;
    overall_goal = LOCALIZATION_GOAL;
    brake_flag = true;
    ROS_WARN("SM: LOCALIZATION SET");
    arm_goal_pub.publish(ARM_GOAL);
  } else if(ALIGNMENT_FLAG)
  {// ALIGN ********************************************************************
    goal_update = true;
    overall_goal = ALIGNMENT_GOAL;
    ROS_WARN("SM: ALIGN SET");
    arm_goal_pub.publish(ARM_GOAL);
  } else if(EXCAVATOR_FLAG)
  {// EXCAVATOR ****************************************************************
    goal_update = true;
    overall_goal = EXCAVATOR_GOAL;
    ROS_WARN("SM: EXCAVATION SET");
    arm_goal_pub.publish(ARM_GOAL);
  } else if(PLANNER_FLAG)
  {// Traverse - planner input *************************************************
    goal_update = true;
    PLANNER_GOAL = overall_goal;
    ROS_WARN("SM: PLANNER SET");
    arm_goal_pub.publish(ARM_GOAL);
  } else
  {// wait *********************************************************************
    brake_srv.request.brake = true;
    brake_client.call(brake_srv);
    brake_flag = true;
    ROS_WARN("SM: WAIT SET");
    PREVIOUS_GOAL.header.stamp = temp_dur;
    PREVIOUS_GOAL.pose.position.x = 0;
    PREVIOUS_GOAL.pose.position.y = 0;
    PREVIOUS_GOAL.pose.position.z = 0;
    PREVIOUS_GOAL.pose.orientation.x = 0;
    PREVIOUS_GOAL.pose.orientation.y = 0;
    PREVIOUS_GOAL.pose.orientation.z = 0;
    PREVIOUS_GOAL.pose.orientation.w = 0;
    arm_goal_pub.publish(ARM_GOAL);
  }

  if(goal_update)
  {
    std::cout << goal_update << std::endl;
    if(brake_flag)
    {
      brake_flag = false;
      brake_srv.request.brake = false;
      brake_client.call(brake_srv);
      ROS_WARN("SM: BRAKES OFF");
    }
    goal_pub.publish(overall_goal);
    ROS_WARN("SM: GOAL SENT");
    PREVIOUS_GOAL = overall_goal;
  }
  goal_update = false;

  rate.sleep();
  ros::spinOnce();
  }


  return 0;
}
