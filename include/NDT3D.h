#ifndef NDT3D_H
#define NDT3D_H

#include "../src/Eigen/Dense"
#include <../src/Eigen/MatrixFunctions>
#include <vector>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class NDT3D {
public:
  /**
  * Constructor.
  */
  NDT3D(VectorXd &pose, const float covC);

  /**
  * Destructor.
  */
  ~NDT3D();

  /**
  * Calculates the normal distribution transform (NDT) for a point cloud relative
  * with a robots pose estimation and returns the robots updated pose estimation.
  */
  VectorXd CalculateNDT(std::vector<std::vector<int>> &ptsCloud);


private:
  VectorXd robotPose; // The estimated position of the robot at a given time.
  float pdfNormConstC;

  /*
  * Calculates the mean of vector q
  * Equation 2 of paper.
  * @param: ptsX => A matrix of cell points that contain and NDT cell.
  * @return: mean of the cell points.
  */
  VectorXd CalcMeanVectorQ(MatrixXd &ptsX);

  /*
  * Calculates the covariance of the NDT cell.
  * Equation 3 of paper.
  * @param: ptsX => A vetor of points that lie within a NDT cell.
  * @param: meanQ => mean of the cell points.
  * @return: covariance matrix of the cell points.
  */
  MatrixXd CalcCovarianceC(MatrixXd &ptsX, VectorXd &meanQ);

  /*
  * Probability density function
  * Equation 4 of paper.
  * @param: ptsX => A vetor of points that lie within a NDT cell.
  * @param: meanQ => mean of the cell points.
  * @param: cov => covariance matrix of the cell points.
  */
  MatrixXd CalcProbDensFunc(VectorXd &ptX, VectorXd &meanQ, MatrixXd &cov);

};

#endif
