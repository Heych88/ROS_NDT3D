#include <iostream>
#include "../include/NDT3D.h"
#include <exception>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

NDT3D::NDT3D(VectorXd& pose, const float covC)
{
  if(pose.size() != 4) // (x, y, z, heading)
    throw std::runtime_error("'pose' incorrect size. Must contain 4 elements.");

  robotPose << pose;
  pdfNormConstC = covC; // normalising constant C
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
* Calculates the mean of vector q
* Equation 2 of paper.
* @param: ptsX => A matrix of cell points that contain and NDT cell.
* @return: mean of the cell points.
*/
VectorXd NDT3D::CalcMeanVectorQ(MatrixXd &ptsX)
{
  VectorXd mean(4);
  mean.fill(0.0);

  for(int i=0; i < ptsX.cols(); i++) {
    mean(0) += ptsX(0,i);
    mean(1) += ptsX(1,i);
    mean(2) += ptsX(2,i);
    mean(3) += ptsX(3,i);
  }

  return mean * (1 / ptsX.cols());
}


/*
* Calculates the covariance of the NDT cell.
* Equation 3 of paper.
* @param: ptsX => A vetor of points that lie within a NDT cell.
* @param: meanQ => mean of the cell points.
* @return: covariance matrix of the cell points.
*/
MatrixXd NDT3D::CalcCovarianceC(MatrixXd &ptsX, VectorXd &meanQ)
{
  MatrixXd covariance(4,4);
  for(int i=0; i < ptsX.cols(); i++) {
    VectorXd pt(4);

    pt(0) = ptsX(0,i) - meanQ(0);
    pt(1) = ptsX(1,i) - meanQ(1);
    pt(2) = ptsX(2,i) - meanQ(2);
    pt(3) = ptsX(3,i) - meanQ(3);

    covariance += pt * pt.transpose();
  }

  return covariance * (1 / (ptsX.size() - 1));
}


/*
* Probability density function
* Equation 4 of paper.
* @param: ptsX => A vetor of points that lie within a NDT cell.
* @param: meanQ => mean of the cell points.
* @param: cov => covariance matrix of the cell points.
*/
MatrixXd NDT3D::CalcProbDensFunc(VectorXd &ptX, VectorXd &meanQ, MatrixXd &cov)
{
  VectorXd vect = ptX - meanQ;
  MatrixXd expon = -(vect.transpose() * cov.inverse() * vect)/2;
  return 1/pdfNormConstC * expon.exp();
}
