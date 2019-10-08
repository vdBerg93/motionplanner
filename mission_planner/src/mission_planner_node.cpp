// Global variables
using namespace std;
// Include STDLIB headers
#include <ros/ros.h>
#include <iostream>
#include <vector>
// Message headers
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
// Headers from RRT package
#include <rrt/functions.h>
#include <rrt/datatypes.h>
#include <rrt/rrtplanner.h>
#include <rrt/vehicle.h>

int main( int argc, char** argv ){	
	// Initialize node
	ros::init(argc, argv, "controls_node");
	// Intialize object for node communication
    ros::NodeHandle nh;
	ros::Rate rate(25);
	
	/*********************************************************************************
		The message manager is responsible for maintaining a motion plan queue.
	 	While the que is not empty, the controller repeatedly pick the first reference 
	 	and executes this untill the end is reached.
	*********************************************************************************/
	while(ros::ok()){
        ros::spinOnce();
	}
}
