## Normal Distribution Transform (NDT)

As per the request the Normal Distribution Transform, here in NDT, was developed as outlined in the paper [here](https://drive.google.com/open?id=1N9-eBLSwOhWH1DwxpH7XbBiF6bE7rPHR). The C++ code presented is a starting implementation of algorithm presented in ROS.

The code as it stands at the end of day two compiles, but performs no functions.
The system was tested on Ubuntu 18.04 with ROS Meliodic.  

## Installation
### Requirments
ROS Melodic. - Installation avaliable [here](http://wiki.ros.org/melodic/Installation/Ubuntu).

## Running the code
1. Clone this repository into the catkin workspace source folder.
```bash
cd ~/catkin_ws/src
git clone https://github.com/Heych88/ROS_NDT3D.git
```

2. Run `catkin_make` from the catkin workspace.
```bash
cd ~/catkin_ws
catkin_make
```

3. Source the project and run the launch file. ***
```bash
source devel/setup.bash
roslaunch norm_dist_tran_3d_node ndt3d.launch
```

*** The project compiles but currently has an Eigen Index Assertions error on launch.


## Project overview
The process taken to produce the code is;
1. Read the relevant paper and documentation provided to gain an understanding of (2 hours);
  * Normal Distribution Transform (NDT) overview.
  * The algorithm psudo code overview
  * Equations that perform the relevant calculations for each part of the psudo code.
  * The elements, parameters and variables of the equations.
  * The input and outputs of the NDT.


2. Produce a plan of action for developing the code. (30mins)
  * Scope the system requirements and the key project focus.
    * Self coded C++ library for mapping vector points using in ROS.
  * Break the development into smaller managable focus sections.
    * NDT
    * ROS
    * Point cloud map
    * NDT point cloud publisher
    * Rviz
  * Brainstorm required components and libraries.
    * Eigen matrix and vector libraries.
    * class
    * ROS messages
  * Strategise plan of attack
    * NDT class compiling in ROS.
    * ROS read and input rosbag of captured point cloud data.
    * Output NDT point cloud
    * ROS to ROS2
    * RViz point cloud


  3. Develope the code. (4.5 hours)
    * Create the ROS project.
    * Create the NDT class.
    * Create Main file /ros node to use the NDT class.
    * Create launch file.
    * Adjust ROS CmakeList.txt and package.xml as required.
    * Provide basic comments.
    * Create GIT repo and push as required.


  4. Produce a short README/Writeup (~1 hour)
    * System outline.
    * Add references and document links.
    * Install and running of the code.
    * Basic project overview.
