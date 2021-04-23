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
#include <volatile_map/VolatileMap.h>


namespace mac {

const int DISTANCE = 0;
const int B = 1;

class CostFunction {
  public:

    /** \brief  */
    inline CostFunction(){cost_type_ = DISTANCE;};
    /** \brief  */
    inline CostFunction(int cost_type)
    : cost_type_(cost_type) {};

    /** \brief  */
    double compute_cost(const volatile_map::Volatile & vol,
                        const mac::Robot    & robot,
                        ros::Time clk) const;

    /** vector is costs[volatile][robot]  */
    std::vector<std::vector<double>> compute_cost_vector(const volatile_map::VolatileMap & volatile_map,
                                            const std::vector<mac::Robot>    & robots,
                                            ros::Time clk) const;
  protected:
    /** \brief  */
    int cost_type_;

    /** \brief  */
    double cost_distance(const volatile_map::Volatile & vol,
                        const mac::Robot    & robot,
                        ros::Time clk) const;

    /** \brief  */
    //double cost_B(const Volatile & volatile,
    //              const Robots   & robot) const {
    //};

    static double dist(const std::vector<double> p1, const std::vector<double> p2);

};
}
#endif // CostFunction_HPP
