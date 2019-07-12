#include <iostream>
#include "../include/NDT3D.h"
#include <exception>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

NDT3D::NDT3D(VectorXd& pose)
{
  if(pose.size() != 3)
    throw std::runtime_error("'pose' incorrect size. Must contain 3 elements.");

  robotPose << pose;

  pdfNormConstC = 1.0;
}

NDT3D::~NDT3D() {}

/*
* Calculates the normal distribution transform (NDT) for a point cloud relative
* with a robots pose estimation and returns the robots updated pose estimation.
* @param: ptsCloud => point cloud data of the environment.
* @return: updated robot pose.
*/
VectorXd NDT3D::CalculateNDT(std::vector<std::vector<int>> &ptsCloud)
{
  robotPose << robotPose * 2;

  return robotPose;
}


/*
* Calculates the mean vector q
* Equation 2 of paper.
*/
float NDT3D::CalcMeanVectorQ(VectorXd &ptsX)
{
  float mean;
  for(int i=0; i < ptsX.size(); i++) {
    mean = mean + ptsX[i];
  }

  return mean + 1 / ptsX.size();
}


/*
* Probability density function
* Equation 4 of paper.
*/
VectorXd NDT3D::CalcProbDensFunc(VectorXd &meanQ, MatrixXd &covC)
{

  return robotPose;
}
