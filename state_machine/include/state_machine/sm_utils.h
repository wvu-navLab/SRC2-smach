
#ifndef SM_UTILS_H
#define SM_UTILS_H

#include <ros/ros.h>

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <cmath>

namespace sm_utils
{

  std::vector<std::pair<double, double>> circlePacking(std::pair<double,double> origin, std::vector<double> P, int size_P, double radius);

}

#endif
