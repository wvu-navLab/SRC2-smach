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


namespace mac {

class CostFunction {
  public:
    /** \brief  */
    inline CostFunction(){};
    /** \brief  */
    inline CostFunction(std::string cost_type)
    : cost_type_(cost_type) {};

    /** \brief  */
    //double compute_cost(const Volatile & volatile,
    //                    const Robot    & robot) const;

  protected:
    /** \brief  */
    const std::string cost_type_;

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
