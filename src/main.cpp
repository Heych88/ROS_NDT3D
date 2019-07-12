#include "../include/NDT3D.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
  // Set up ROS.
  ros::init(argc, argv, "norm_dist_tran_3d_node");

  std::vector<std::vector<int>> ptsCloud (5, std::vector<int>(5,0));

  VectorXd botPose(3);
  botPose << 7.2, 1.1, 22.7;

  NDT3D botNDT(botPose);

  botPose = botNDT.CalculateNDT(ptsCloud);

  return 0;
}
