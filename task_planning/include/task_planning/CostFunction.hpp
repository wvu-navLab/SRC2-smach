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

namespace mac
{

  const int DISTANCE = 0;
  const int POWER = 1;
  const int UNCERTAINTY_ROBOT = 2;
  const int UNCERTAINTY_VOLATILE = 3;
  const int TIME_REMAINING = 4;
  const int SWITCH_TASK = 5;
  const int G = 6;
  const int H = 7;

  class CostFunction
  {
  public:
    /** \brief  */
    inline CostFunction() { cost_type_ = DISTANCE; };
    /** \brief  */
    inline CostFunction(int cost_type)
        : cost_type_(cost_type){};

    /** \brief  */
    double compute_cost(const State s,
                        const std::vector<Action> joint_action,
                        const State s_prime) const;

    /** vector is costs[volatile][robot]  */
    std::vector<std::vector<double>> compute_cost_vector(const State s,
                                                         const std::vector<Action> joint_action,
                                                         const State s_prime) const;

  protected:
    /** \brief  */
    int cost_type_;

    /** \brief  */
    double cost_distance(const State s,
                         const std::vector<Action> joint_action,
                         const State s_prime) const;

    /** \brief  */
    double cost_power(const State s,
                      const std::vector<Action> joint_action,
                      const State s_prime) const;

    /** \brief  */
    double cost_unc_robot(const State s,
                          const std::vector<Action> joint_action,
                          const State s_prime) const;

    /** \brief  */
    double cost_unc_volatile(const State s,
                             const std::vector<Action> joint_action,
                             const State s_prime) const;

    /** \brief  */
    double cost_time_remaining(const State s,
                               const std::vector<Action> joint_action,
                               const State s_prime) const;

    /** \brief  */
    double cost_switch_task(const State s,
                            const std::vector<Action> joint_action,
                            const State s_prime) const;

    /** \brief  */
    double cost_G(const State s,
                  const std::vector<Action> joint_action,
                  const State s_prime) const;

    /** \brief  */
    double cost_H(const State s,
                  const std::vector<Action> joint_action,
                  const State s_prime) const;

    /** \brief  */
    //double cost_B(const Volatile & volatile,
    //              const Robots   & robot) const {
    //};

    static double dist(const std::vector<double> p1, const std::vector<double> p2);
  };
}
#endif // CostFunction_HPP
