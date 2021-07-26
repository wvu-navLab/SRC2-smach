cmake_minimum_required(VERSION 3.0.2)
project(state_machine)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_runtime
  roslib
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  driving_tools
  # waypoint_nav
  waypoint_gen
  srcp2_msgs
  nav_msgs
  move_excavator
  actionlib
  sensor_fusion
  src2_object_detection
  std_srvs
  waypoint_checker
  message_generation
  motion_control
  message_filters
  tf2_ros
  dynamic_reconfigure
  volatile_map
  task_planning
  laser_tools_src2
  range_to_base
  src2_approach_services
  waypoint_nav
  localization_watchdog
)

find_package(Eigen3 3.3 REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  RobotStatus.msg
  ExcavationStatus.msg
  HaulerStatus.msg
)

# Generate services in the 'srv' folder
 add_service_files(
   FILES
   SetMobility.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES state_machine
  CATKIN_DEPENDS
  srcp2_msgs
  sensor_msgs
  waypoint_checker
  src2_object_detection
  src2_approach_services
  waypoint_nav
  std_msgs std_srvs
  geometry_msgs
  nav_msgs
  sensor_fusion
  driving_tools
  actionlib
  move_excavator
  waypoint_gen
  motion_control
  message_filters
  tf2_ros
  dynamic_reconfigure
  volatile_map
  task_planning
  localization_watchdog
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ~/ros_workspace/install/include
 ~/cmp_workspace/src/SRC2-smach/task_planning/include
 ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(sm_utils
  utils/sm_utils.cpp
)

 add_library(sm_scout
   src/sm_scout.cpp
 )
add_library(sm_hauler
   src/sm_hauler.cpp
)
add_library(sm_excavator
   src/sm_excavator.cpp
)



#  add_library(sm_rd3
#    src/sm_rd3.cpp
 #)
## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(sm_utils ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(sm_scout ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(sm_hauler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(sm_excavator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(sm_utils
  ${catkin_LIBRARIES}
)

target_link_libraries(sm_scout
  ${catkin_LIBRARIES}
)
target_link_libraries(sm_hauler
  ${catkin_LIBRARIES}
)

target_link_libraries(sm_excavator
  ${catkin_LIBRARIES}
)
 #target_link_libraries(sm_rd3
#   ${catkin_LIBRARIES}
 #)
## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/state_machine_node.cpp)
add_executable(utils_test_node utils/utils_test_node.cpp)

add_executable(state_machine_scout_node src/sm_scout_node.cpp)

add_executable(state_machine_hauler_node src/sm_hauler_node.cpp)

add_executable(state_machine_excavator_node src/sm_excavator_node.cpp)


## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(state_machine_scout_node state_machine_generate_messages_cpp)
add_dependencies(state_machine_hauler_node state_machine_generate_messages_cpp)
add_dependencies(state_machine_excavator_node state_machine_generate_messages_cpp)

add_dependencies(utils_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(state_machine_scout_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(state_machine_hauler_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(state_machine_excavator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(utils_test_node
  ${catkin_LIBRARIES}
  sm_utils
)

 target_link_libraries(state_machine_scout_node
   ${catkin_LIBRARIES}
   sm_scout
 )

target_link_libraries(state_machine_hauler_node
  ${catkin_LIBRARIES}
  sm_hauler
)
target_link_libraries(state_machine_excavator_node
  ${catkin_LIBRARIES}
  sm_excavator
)
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
 install(TARGETS sm_scout sm_utils sm_hauler sm_excavator
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
 )
install(TARGETS state_machine_scout_node state_machine_hauler_node state_machine_excavator_node utils_test_node
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
   PATTERN ".svn" EXCLUDE)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_state_machine.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
