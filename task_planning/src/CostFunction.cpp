// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file CostFunction.cpp
 */
//-----------------------------------------------------------------------------

#include <task_planning/CostFunction.hpp>

namespace mac {

double CostFunction::
compute_cost(const Volatile & volatile_objective,
             const Robot    & robot) const {

  switch(cost_type_)
  {
    case A :
      //do stuff
      break;

    case B:
      //do stuff
      break;

    default:
      ROS_ERROR("CostFunction::compute_cost: cost_type invalid!");
      break;
  }

};

}
