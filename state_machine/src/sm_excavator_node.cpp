/**
@file example_sm_node.cpp
Runs the Excavator state machine for the SRC2 competition
@author Jared Beard, Nick Ohi, Bernardo Martinez
@version 1.0 7/5/2020
*/

#include <state_machine/sm_excavator.hpp>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "state_machine_excavator_node");
  SmExcavator obj;
  obj.run();

  return 0;
}


