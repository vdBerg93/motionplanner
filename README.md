# ROS Motion Planner Package

This package contains node for implementing the motion planner in a mobile robot.
Package contains the following nodes:
* pcl_converter: convert pointcloud into 2D Oriented Bounding Boxes
* rrt: Generate a motion plan using RRT with closed-loop prediction
* controls: Make the mobile robot follow the planned motion

## Getting Started

### Prerequisites

```
Give examples
```

### Installing

```
mkdir ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/vdBerg93/motionplanner.git
catkin_make
roslaunch rrt rrt.launch
```
## Running the tests

Explain how to run the automated tests for this system

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
