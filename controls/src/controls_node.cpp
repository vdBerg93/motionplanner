// Include STDLIB headers
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <array>
#include <cstdlib>
#include <cmath>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

using namespace std;
double ctrl_dla, tla, ref_res;

#include <rrt/functions.h>
#include <rrt/datatypes.h>
#include <rrt/rrtplanner.h>
#include <rrt/vehicle.h>

//#include "control.cpp"

int main( int argc, char** argv ){	
	// std::cout.precision(2);
	// // MyReference ref;
	// MyReference ref;
	// // Initialize ros node handle
	// ros::init(argc, argv, "controls_node");
	// ros::NodeHandle nh; ros::Rate rate(2);

	// // Register localization subsriber
	// // ros::Subscriber subPos = nh.subscribe();

	// // Register motion speficication subsriber
	// // ros::Subscriber subPos = nh.subscribe();
	
	// // Register controls publisher
	// // ros::Publisher pubCtrl = nh.advertise<...>("controls",10);

	// while(ros::ok()){
	// 	ros::spin();
	// }
}
