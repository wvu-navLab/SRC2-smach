/**
@file example_sm_node.cpp
Runs the round 2 state machine for the SRC2 competition
@author Jared Beard, Nick Ohi, Bernardo Rocamora Martinez Junior, Cagri Kilic
@version 1.0 8/20/2020
*/

#include <state_machine/sm_rd2.hpp>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "state_machine_rd2_node");
  SmRd2 obj;
  obj.run();

  return 0;
}
