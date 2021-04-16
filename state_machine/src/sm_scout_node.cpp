/**
@file example_sm_node.cpp
Runs the round 1 state machine for the SRC2 competition
@author Jared Beard, Nick Ohi
@version 1.0 7/5/2020
*/

#include <state_machine/sm_scout.hpp>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "state_machine_rd1_node");
  SmScout obj;
  obj.run();

  return 0;
}


