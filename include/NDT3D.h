#ifndef NDT3D_H
#define NDT3D_H

#include "../src/Eigen3/Dense"
#include <vector>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class NDT3D {
public:
  /**
  * Constructor.
  */
  NDT3D(VectorXd &pose);

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
  * Calculates the mean vector q
  * Equation 2 of paper.
  */
  float CalcMeanVectorQ(VectorXd &ptsX);

  /*
  * Probability density function
  * Equation 4 of paper.
  */
  VectorXd CalcProbDensFunc(VectorXd &meanQ, MatrixXd &covC);

};

#endif
