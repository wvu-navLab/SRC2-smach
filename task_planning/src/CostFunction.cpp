// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file CostFunction.cpp
 */
//-----------------------------------------------------------------------------

#include <task_planning/CostFunction.hpp>

namespace mac
{

  double CostFunction::compute_cost(const State s,
                                    const std::vector<Action> joint_action,
                                    const State s_prime) const
  {

    switch (cost_type_)
    {
    case DISTANCE:
      return this->cost_distance(s, joint_action, s_prime);

    case POWER:
      return this->cost_power(s, joint_action, s_prime);

    case UNCERTAINTY_ROBOT:
      return this->cost_unc_robot(s, joint_action, s_prime);

    case UNCERTAINTY_VOLATILE:
      return this->cost_unc_volatile(s, joint_action, s_prime);

    case TIME_REMAINING:
      return this->cost_time_remaining(s, joint_action, s_prime);

    case SWITCH_TASK:
      return this->cost_switch_task(s, joint_action, s_prime);

    case VOL_COLLECTED:
      return this->cost_vol_collected(s, joint_action, s_prime);

    case TIME_PASSED:
      return this->cost_time_passed(s, joint_action, s_prime);

    case TIME_VOL_COLLECTED:
      return this->cost_time_vol_collected(s, joint_action, s_prime);

    default:
      ROS_ERROR("CostFunction::compute_cost: cost_type invalid!");
      break;
    }
    return 0.0;
  }

  std::vector<std::vector<double>> CostFunction::compute_cost_vector(const State s,
                                                                     const std::vector<Action> joint_action,
                                                                     const State s_prime) const
  {
    std::vector<std::vector<double>> costs;
    // for (int i = 0; i < volatile_map.vol.size(); ++i)
    // {
    //   std::vector<double> temp;
    //   for (mac::Robot rbt : robots)
    //     temp.push_back(compute_cost(volatile_map.vol[i], rbt, clk));
    //   costs.push_back(temp);
    // }
    return costs;
  }

  double CostFunction::cost_distance(State s,
                                     const std::vector<Action> joint_action,
                                     const State s_prime) const
  {
    // std::vector<double> vol_pose({vol.position.point.x, vol.position.point.y});
    // std::vector<double> robot_pose({robot.odom.pose.pose.position.x, robot.odom.pose.pose.position.y});

    // return CostFunction::dist(vol_pose, robot_pose);
    return 0;
  }

  double CostFunction::cost_power(const State s,
                                  const std::vector<Action> joint_action,
                                  const State s_prime) const
  {
    return 0;
  }

  double CostFunction::cost_unc_robot(const State s,
                                      const std::vector<Action> joint_action,
                                      const State s_prime) const
  {
    return 0;
  }

  double CostFunction::cost_unc_volatile(const State s,
                                         const std::vector<Action> joint_action,
                                         const State s_prime) const
  {
    return 0;
  }

  double CostFunction::cost_time_remaining(const State s,
                                           const std::vector<Action> joint_action,
                                           const State s_prime) const
  {
    return 0;
  }

  double CostFunction::cost_switch_task(const State s,
                                        const std::vector<Action> joint_action,
                                        const State s_prime) const
  {
    return 0;
  }

  double CostFunction::cost_vol_collected(const State s,
                                          const std::vector<Action> joint_action,
                                          const State s_prime) const
  {
    double cost = 0;
    for (int i = 0; i < s.volatile_map.vol.size(); ++i)
    {
      if (s_prime.volatile_map.vol[i].collected && !s.volatile_map.vol[i].collected)
      {
        cost += get_vol_value(s_prime.volatile_map.vol[i].type);
      }
    }

    return 0;
  }

  double CostFunction::cost_time_passed(const State s,
                                        const std::vector<Action> joint_action,
                                        const State s_prime) const
  {
    //maybe this doesn't work if changed task, so probably want to do checking... may check that it isn't zero
    return params_.time_weight * (s.robots[0].time_remaining - s_prime.robots[0].time_remaining);
  }

  double CostFunction::cost_time_vol_collected(const State s,
                                               const std::vector<Action> joint_action,
                                               const State s_prime) const
  {
    double time_cost = cost_time_passed(s, joint_action, s_prime);
    double vol_cost = cost_vol_collected(s, joint_action, s_prime);
    return params_.time_vol_weights[0] * time_cost + params_.time_vol_weights[1] * vol_cost;
  }

  double CostFunction::dist(const std::vector<double> p1, const std::vector<double> p2)
  {
    if (p1.size() != p2.size())
    {
      std::cout << "Error! p1.size() != p2.size() for computing distance!\n";
      exit(1); //TODO: remove exit
    }
    double val = 0;
    for (int i = 0; i < p1.size(); i++)
    {
      double diff = p1[i] - p2[i];
      val = val + diff * diff;
    }
    val = std::sqrt(val);
    return val;
  }

  double CostFunction::get_vol_value(std::string vol)
  {
    if (vol.compare("ice"))
    {
      return 1; //points /kg
    }
    else if (vol.compare("ethane"))
    {
      return 20;
    }
    else if (vol.compare("methane"))
    {
      return 14;
    }
    else if (vol.compare("methanol"))
    {
      return 16;
    }
    else if (vol.compare("carbon_dioxide"))
    {
      return 3;
    }
    else if (vol.compare("ammonia"))
    {
      return 2;
    }
    else if (vol.compare("hydrogen_sulfite"))
    {
      return 1;
    }
    else if (vol.compare("sulfur_dioxide"))
    {
      return 1;
    }
    else if (vol.compare("regolith"))
    {
      return 0;
    }
    else
    {
      ROS_ERROR("Invalid volatile type: get_vol_value");
      return -1;
    }
  }
}
