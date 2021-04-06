// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file CostFunction.hpp
 */
//-----------------------------------------------------------------------------

#ifndef CostFunction_HPP
#define CostFunction_HPP

#include <ros/ros.h>

#include <rosgraph_msgs/Clock.h>
#include <task_planning/Types.hpp>


namespace mac {

const int A = 0;
const int B = 1;

class CostFunction {
  public:

    /** \brief  */
    inline CostFunction(){cost_type_ = A;};
    /** \brief  */
    inline CostFunction(int cost_type)
    : cost_type_(cost_type) {};

    /** \brief  */
    double compute_cost(const Volatile & volatile_objective,
                        const Robot    & robot) const;

  protected:
    /** \brief  */
    int cost_type_;

    /** \brief  */
    //double cost_A(const Volatile & volatile,
    //              const Robots   & robot) const {
    //};

    /** \brief  */
    //double cost_B(const Volatile & volatile,
    //              const Robots   & robot) const {
    //};
};

}

#endif // CostFunction_HPP
