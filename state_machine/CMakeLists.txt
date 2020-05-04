cmake_minimum_required(VERSION 2.8.3)
project(state_machine)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  genmsg
  actionlib_msgs
  std_msgs
  sensor_msgs
)

catkin_python_setup()

 add_service_files(
   FILES
   Init.srv
   LegSelect.srv
   Shutdown.srv
 )

 add_action_files(
   FILES
   ApproachMarker.action
   CircleMarker.action
   CommsLoss.action
   GPSLoss.action
   GPSNav.action
   Obav.action
   OptimalSearch.action
   SignalCompletion.action
   TeleOp.action
 )

 generate_messages(
   DEPENDENCIES
   actionlib_msgs
   std_msgs
   sensor_msgs
 )

catkin_package(
	CATKIN_DEPENDS
  message_runtime
  roscpp
  rospy
  actionlib_msgs
  std_msgs
  sensor_msgs
)
