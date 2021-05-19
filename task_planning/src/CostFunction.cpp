// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file CostFunction.cpp
 */
//-----------------------------------------------------------------------------

#include <task_planning/CostFunction.hpp>

namespace mac {

double CostFunction::compute_cost(const volatile_map::Volatile & vol,
                    const mac::Robot    & robot,
                    ros::Time clk) const {

  switch(cost_type_)
  {
    case DISTANCE :
      return this->cost_distance(vol, robot, clk);

    case POWER:
      return this->cost_power(vol, robot, clk);

    case UNCERTAINTY_ROBOT:
      return this->cost_unc_robot(vol, robot, clk);

    case UNCERTAINTY_VOLATILE:
      return this->cost_unc_volatile(vol, robot, clk);

    case TIME_REMAINING:
      return this->cost_time_remaining(vol, robot, clk);

    case SWITCH_TASK:
      return this->cost_switch_task(vol, robot, clk);

    case G:
      return this->cost_G(vol, robot, clk);

    case H:
      return this->cost_H(vol, robot, clk);

    default:
      ROS_ERROR("CostFunction::compute_cost: cost_type invalid!");
      break;
  }
  return 0.0;
}

std::vector<std::vector<double>> CostFunction::compute_cost_vector(const volatile_map::VolatileMap & volatile_map,
                                        const std::vector<mac::Robot>    & robots,
                                        ros::Time clk) const {
  std::vector<std::vector<double>> costs;
  for (int i = 0 ; i < volatile_map.vol.size(); ++ i){
    std::vector<double> temp;
    for (mac::Robot rbt : robots) temp.push_back(compute_cost(volatile_map.vol[i],rbt,clk));
    costs.push_back(temp);
  }
  return costs;
}

double CostFunction::cost_distance(const volatile_map::Volatile & vol,
                    const mac::Robot    & robot,
                    ros::Time clk) const {
  std::vector<double> vol_pose({vol.position.point.x, vol.position.point.y});
  std::vector<double> robot_pose({robot.odom.pose.pose.position.x, robot.odom.pose.pose.position.y});

  return CostFunction::dist(vol_pose, robot_pose);
}

double CostFunction::dist(const std::vector<double> p1, const std::vector<double> p2) {
if(p1.size() != p2.size())
{
std::cout << "Error! p1.size() != p2.size() for computing distance!\n";
exit(1); //TODO: remove exit
}
double val=0;
for(int i=0; i<p1.size(); i++)
{
  double diff = p1[i] - p2[i];
  val = val + diff*diff;
}
val = std::sqrt(val);
return val;
}

double CostFunction::cost_power(const volatile_map::Volatile & vol,
                                const mac::Robot             & robot,
                                      ros::Time clk) const
{
  return 0;
}


double CostFunction::cost_unc_robot(const volatile_map::Volatile & vol,
                                const mac::Robot             & robot,
                                      ros::Time clk) const
{
  return 0;
}


double CostFunction::cost_unc_volatile(const volatile_map::Volatile & vol,
                                const mac::Robot             & robot,
                                      ros::Time clk) const
{
  return 0;
}


double CostFunction::cost_time_remaining(const volatile_map::Volatile & vol,
                                const mac::Robot             & robot,
                                      ros::Time clk) const
{
  return 0;
}


double CostFunction::cost_switch_task(const volatile_map::Volatile & vol,
                                const mac::Robot             & robot,
                                      ros::Time clk) const
{
  return 0;
}


double CostFunction::cost_G(const volatile_map::Volatile & vol,
                                const mac::Robot             & robot,
                                      ros::Time clk) const
{
  return 0;
}


double CostFunction::cost_H(const volatile_map::Volatile & vol,
                    const mac::Robot    & robot,
                    ros::Time clk) const
{

}

}
