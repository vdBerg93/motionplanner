# ROS Motion Planner Package

## To do
* Make node for state estimation
* Add function for splitting node into multiple smaller sections
* Add variable sampling domain

## Getting Started

This package contains node for implementing the motion planner in a mobile robot.
Package contains the following nodes:
* obstacle_tracker: track pedestrian trough world and esimate its velocity
* rrt: Generate a motion plan using RRT with closed-loop prediction
* state_estimator: generates state estimation message
* simulator: contains a Toyota Prius Gazebo simulator
* car_msgs: contains all messages used by the package

### Prerequisites

```
ROS Melodic with Visualization stack

```

### Installing

```
mkdir ~/catkin_ws/src
cd ~/catkin_ws/src

git clone -b newcostfunction https://github.com/vdBerg93/motionplanner.git
git clone -b four_persons https://github.com/vdBerg93/pedsim_ros.git
git clone https://github.com/bbrito/lmpcc_msgs.git
git clone -b rrt https://gitlab.tudelft.nl/bdebrito/lmpcc_obstacle_feed.git 
git clone -b rrt_integration https://gitlab.tudelft.nl/bdebrito/lmpcc.git

catkin_make
```
## Running the tests
Two scenarios can be tested:
1. pedwalking.launch
2. pedcrossing.launch

The following code will start the simulations:

```
roslaunch rrt #scenario#
roslaunch lmpcc_obstacle_feed lmpcc_obstacle_feed.launch
roslaunch lmpcc lmpcc.launch
roslaunch rrt rrt.launch
```
### Break down into end to end tests

Explain what these tests test and why

```
Give an example
```

### And coding style tests

Explain what these tests test and why

```
Give an example
```

## Deployment

Add additional notes about how to deploy this on a live system

## Authors

* **Berend van den Berg** - *Initial work* - [vdBerg93](https://github.com/vdBerg93)

## Acknowledgments

* 
