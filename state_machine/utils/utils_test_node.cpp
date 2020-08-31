
#include <ros/ros.h>
#include <state_machine/sm_utils.h>

#include <iostream>
#include <vector>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "state_machine_rd1_node");

  std::vector<std::pair<double, double>>  coordinates;

  std::pair<double,double> origin({1, 2});
  std::vector<double> P({1.0, 2.0, 2.0, 2.5});
  int size_P = 2;
  double radius = 0.5;

  coordinates = sm_utils::circlePacking( origin, P, size_P, radius);

  for (int i = 0; i < coordinates.size(); ++i)
  {
    std::cout << coordinates[i].first << ", " << coordinates[i].second << "; " ;
  }

  return 0;
}
