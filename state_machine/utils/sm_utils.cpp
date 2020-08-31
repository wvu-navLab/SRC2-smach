/**
@file sm_utils.cpp
Utility functions for
@author Jared Beard,
@version 1.0 8/25/2020
*/

#include <state_machine/sm_utils.h>

namespace sm_utils
{

std::vector<std::pair<double, double>> circlePacking(std::pair<double,double> origin, std::vector<double> P, int size_P, double radius)
{
    //variables

    std::vector<std::pair<double, double>> samples, transformed_samples;
    Eigen::MatrixXd P_mat = Eigen::MatrixXd::Zero(size_P,size_P);
    int count = 0;
    Eigen::Vector2d origin_vec(origin.first,origin.second);
    /**Get transform of P matrix Eigenvalues*/
    // Convert to matrix
    for (int i = 0; i < size_P; ++i)
    {
      for(int j = 0; j < size_P; ++j)
      {
        P_mat(i,j) = P[count];
        ++count;
      }
    }

    // Evaluate eigenvectors/values
    //ROS_INFO_STREAM(P_mat(0,0) << "," << P_mat(0,1));
    //ROS_INFO_STREAM(P_mat(1,0) << "," << P_mat(1,1));
    Eigen::EigenSolver<Eigen::MatrixXd> eigSolver(P_mat);
    Eigen::VectorXd eigVector;
    eigVector = eigSolver.eigenvectors().col(0).real();
    /**for (int i = 0; i < 2; ++i)
    {
      //ROS_INFO_STREAM(eigSolver.eigenvectors().col(i).real());
    }*/
    Eigen::VectorXd eigValues;
    eigValues = eigSolver.eigenvalues().real();
    /**for (int i = 0; i < 2; ++i)
    {
      ROS_INFO_STREAM(eigValues[i]);
    }*/
    //Eigen::Vector2d eigValues(eigSolver.eigenvectors().cols(0));
    std::vector<double> scaledEigenVector;
    for (int i = 0; i < size_P; ++i)
    {
      scaledEigenVector.push_back(eigVector(i)*eigValues(0));
      //ROS_INFO_STREAM(scaledEigenVector[i]);
    }
    //Pack in transformed space
    double ellipseRadius = 0;
    double x = 0.0, y = 0.0;
    int n = 0, m = 0;
    std::pair<double,double> temp_pair;
    //aligned points

    while (x < scaledEigenVector[0])
    {
      while (ellipseRadius <= 1.0)
      {
        temp_pair = {x,y};
        transformed_samples.push_back(temp_pair);
        if (y > 0)
        {
          temp_pair = {x,-y};
          transformed_samples.push_back(temp_pair);

          if (x > 0)
          {
            temp_pair = {-x,y};
            transformed_samples.push_back(temp_pair);
            temp_pair = {-x,-y};
            transformed_samples.push_back(temp_pair);
          }
        }
        ++n;
        y = n*sqrt(5)*radius;
        ellipseRadius = (x*x)/pow(scaledEigenVector[0],2) + (y*y)/pow(scaledEigenVector[1],2);
      }
      ++m;
      x = m*radius;
      y = sqrt(5)*radius;
      ellipseRadius = 0;
    }

    x = radius/2.0;
    y = sqrt(5)/2.0*radius;
    n = 0;
    m = 0;
    //offset points
    //ROS_INFO("1");
    while (ellipseRadius <= 1.0)
    {
      ellipseRadius = 0;
      while (ellipseRadius <= 1.0)
      {
        temp_pair = {x,y};
        transformed_samples.push_back(temp_pair);
        temp_pair = {x,-y};
        transformed_samples.push_back(temp_pair);

        temp_pair = {-x,y};
        transformed_samples.push_back(temp_pair);
        temp_pair = {-x,-y};
        transformed_samples.push_back(temp_pair);

        ++n;
        y = n*sqrt(5)/2.0*radius;
        ellipseRadius = (x*x)/pow(scaledEigenVector[0],2) + (y*y)/pow(scaledEigenVector[1],2);
        ROS_INFO_STREAM(ellipseRadius);
      }
      m+=2;
      x = ( (m+1.0)/2.0 )*radius;
      y = sqrt(5)/2.0*radius;

    }

    //transform to normal space
    double theta = atan2(scaledEigenVector[1],scaledEigenVector[0]);
    Eigen::Matrix2d R;
    R << cos(theta), -sin(theta), sin(theta), cos(theta);
    ROS_INFO_STREAM(theta);
    ROS_INFO_STREAM(R);
    for (int i = 0; i < transformed_samples.size();++i)
    {
      Eigen::Vector2d temp_vec;
      temp_vec << transformed_samples[i].first, transformed_samples[i].second;
      Eigen::Vector2d point = R.inverse()*temp_vec + origin_vec;
      std::pair<double,double> temp_pair(point(0),point(1));
      samples.push_back(temp_pair);
    }


    return samples;
}


}
