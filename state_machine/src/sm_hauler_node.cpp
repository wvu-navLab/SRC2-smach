/**
@file sm_hauler_node.cpp
Runs the Hauler state machine for the SRC2 final competition
@author Cagri Kilic
@version 1.0 April 16, 2021
*/

#include <state_machine/sm_hauler.hpp>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "state_machine_hauler_node");
  SmHauler obj;
  obj.run();

  return 0;
}
